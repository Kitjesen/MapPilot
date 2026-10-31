"""Build a non-runnable, explicitly derived ELF analysis projection.

Preserves byte-identical selected functions/data at original virtual addresses.
No code patching, execution or invented implementations. Unselected callees and
indirect targets are outside coverage. Symbols come from matching debug sidecar.
"""
import argparse
import hashlib
import json
import mmap
import struct
from pathlib import Path
from elftools.elf.elffile import ELFFile

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--source", type=Path, required=True)
parser.add_argument("--reports", type=Path, nargs="+", required=True)
parser.add_argument("--output", type=Path, required=True)
args = parser.parse_args()
core = args.source
output = args.output
manifest_path = output.with_suffix(".manifest.json")
reports = {}
for directory in args.reports:
    for path in sorted(directory.glob("0x*.json")):
        item = json.loads(path.read_text())
        reports[int(item["address"], 16)] = item
selected = set(reports)
regions = []
symbols = []
with core.open("rb") as stream, Path(str(core)+".debug").open("rb") as ds:
    elf, debug = ELFFile(stream), ELFFile(ds)
    table = debug.get_section_by_name(".symtab")
    names = debug.get_section(table["sh_link"]).data()
    raw_symbols = table.data()
    for name, info, other, section, address, size in struct.iter_unpack("<IBBHQQ", raw_symbols):
        if address in selected and info & 15 == 2 and size:
            end = names.find(b"\0", name)
            if not any(s[0] == address for s in symbols):
                symbols.append((address, size, names[name:end].decode()))
                regions.append((address, address+size, 5, "function"))
    if {s[0] for s in symbols} != selected:
        raise RuntimeError("A selected function has no exact debug boundary")
    for report in reports.values():
        for ref in report["rip_data"]:
            address = int(ref["address"], 16)
            regions.append((address, address+len(bytes.fromhex(ref["hex"])), 4, "data-candidate"))
    plt = elf.get_section_by_name(".plt")
    regions.append((plt["sh_addr"], plt["sh_addr"]+plt["sh_size"], 5, "plt"))
    rela = elf.get_section_by_name(".rela.plt")
    dyn = elf.get_section(rela["sh_link"])
    for index, rel in enumerate(rela.iter_relocations()):
        symbols.append((plt["sh_addr"]+16*(index+1),16,dyn.get_symbol(rel["r_info_sym"]).name))
    merged = []
    for start,end,flags,kind in sorted(regions):
        if merged and start <= merged[-1][1]:
            previous = merged[-1]
            merged[-1] = (previous[0], max(end,previous[1]),flags|previous[2],previous[3]+[kind])
        else:
            merged.append((start,end,flags,[kind]))
    loads = [s for s in elf.iter_segments() if s["p_type"] == "PT_LOAD"]
    def source_offset(address,length):
        for segment in loads:
            delta = address-segment["p_vaddr"]
            if 0 <= delta and delta+length <= segment["p_filesz"]:
                return segment["p_offset"]+delta
        raise RuntimeError(f"Selected range is not file-backed: {address:#x}+{length}")
    sections = []
    mappings = []
    with mmap.mmap(stream.fileno(),0,access=mmap.ACCESS_READ) as original:
        source_sha = hashlib.sha256(original).hexdigest()
        for start,end,flags,kinds in merged:
            pos = source_offset(start,end-start)
            data = original[pos:pos+end-start]
            sections.append({"name":f".fragment_{start:x}","type":1,"flags":2|(4 if flags&1 else 0),
                             "address":start,"bytes":data,"align":1,"link":0,"info":0,"entsize":0})
            mappings.append({"virtual_address":hex(start),"length":len(data),"source_file_offset":pos,
                             "sha256":hashlib.sha256(data).hexdigest(),"kinds":sorted(set(kinds))})
    names_blob=bytearray(b"\0")
    symbol_bytes=bytearray(bytes(24))
    for address,size,name in sorted(symbols):
        string_offset=len(names_blob)
        names_blob.extend(name.encode()+b"\0")
        section_index=next(i+1 for i,s in enumerate(sections) if s["address"]<=address<s["address"]+len(s["bytes"]))
        symbol_bytes.extend(struct.pack("<IBBHQQ",string_offset,0x12,0,section_index,address,size))
    sym_index=len(sections)+1
    str_index=sym_index+1
    sections.extend([
        {"name":".symtab","type":2,"flags":0,"address":0,"bytes":bytes(symbol_bytes),"align":8,"link":str_index,"info":1,"entsize":24},
        {"name":".strtab","type":3,"flags":0,"address":0,"bytes":bytes(names_blob),"align":1,"link":0,"info":0,"entsize":0}])
    shstr=bytearray(b"\0")
    for sec in sections:
        sec["name_offset"]=len(shstr)
        shstr.extend(sec["name"].encode()+b"\0")
    shstr_offset=len(shstr)
    shstr.extend(b".shstrtab\0")
    sections.append({"name":".shstrtab","name_offset":shstr_offset,"type":3,"flags":0,"address":0,
                     "bytes":bytes(shstr),"align":1,"link":0,"info":0,"entsize":0})
    def align(value,alignment):
        return (value+alignment-1)//alignment*alignment
    phnum=len(merged)
    cursor=64+phnum*56
    for sec in sections:
        cursor=align(cursor,sec["align"])
        sec["offset"]=cursor
        cursor+=len(sec["bytes"])
    shoff=align(cursor,8)
    image=bytearray(shoff+64*(len(sections)+1))
    ident=b"\x7fELF\x02\x01\x01"+bytes(9)
    # ET_EXEC addresses preserved; entry=0 deliberately. This is not a product binary.
    image[:64]=struct.pack("<16sHHIQQQIHHHHHH",ident,2,62,1,0,64,shoff,0,64,56,phnum,64,len(sections)+1,len(sections))
    for index,sec in enumerate(sections[:phnum]):
        image[64+56*index:64+56*(index+1)]=struct.pack("<IIQQQQQQ",1,5 if sec["flags"]&4 else 4,
            sec["offset"],sec["address"],sec["address"],len(sec["bytes"]),len(sec["bytes"]),1)
    for index,sec in enumerate(sections,1):
        image[sec["offset"]:sec["offset"]+len(sec["bytes"])]=sec["bytes"]
        image[shoff+index*64:shoff+(index+1)*64]=struct.pack("<IIQQQQIIQQ",sec["name_offset"],sec["type"],
            sec["flags"],sec["address"],sec["offset"],len(sec["bytes"]),sec["link"],sec["info"],sec["align"],sec["entsize"])
    with output.open("xb") as dest:
        dest.write(image)
    with output.open("rb") as check, core.open("rb") as source:
        projection=ELFFile(check)
        for mapping in mappings:
            address=int(mapping["virtual_address"],16)
            segment=next(s for s in projection.iter_segments() if s["p_vaddr"]==address)
            check.seek(segment["p_offset"])
            source.seek(mapping["source_file_offset"])
            if check.read(mapping["length"]) != source.read(mapping["length"]):
                raise RuntimeError("Projection changed source bytes")
    manifest={"format":"matrix-derived-analysis-projection-v1","source_path":str(core),
        "source_sha256":source_sha,"output_sha256":hashlib.sha256(image).hexdigest(),"output_bytes":len(image),
        "function_count":len(selected),"ranges":mappings,
        "symbols":[{"address":hex(a),"size":s,"name":n} for a,s,n in sorted(symbols)],
        "verification":"All mapped initialized ranges compared byte-for-byte with verified source.",
        "limitations":["Derived analysis ELF, not original UeSim and not runnable.",
            "Only selected functions, candidate data windows and original PLT are covered.",
            "Original virtual addresses retained; ELF headers/sections/symbol table are newly generated.",
            "Imported symbol assertions from official matching debug sidecar; not Ghidra discoveries.",
            "Original dynamic linker metadata, unselected code, runtime memory, and indirect call types omitted.",
            "Decompiled pointer types/callee signatures remain hypotheses; disassembly is authoritative."]}
    with manifest_path.open("x") as dest:
        json.dump(manifest,dest,indent=2)
    print(json.dumps({"output_bytes":len(image),"functions":len(selected),"ranges":len(mappings),
                      "source_sha256":source_sha,"output_sha256":manifest["output_sha256"]}),flush=True)
