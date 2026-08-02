import json
import yaml

manifest = json.load(open('.tmp_manifest.json', encoding='utf-8'))
contract = yaml.safe_load(open('config/topic_contract.yaml', encoding='utf-8'))
keys = [
    'topic_allowed_frame_ids',
    'topic_default_frame_ids',
    'real_runtime_topic_allowed_frame_ids',
    'real_runtime_topic_default_frame_ids',
    'topic_formats',
    'topic_ros_types',
]
for k in keys:
    m = manifest.get(k, {})
    c = contract.get(k, {})
    if m == c:
        print(k, 'SAME')
        continue
    print('---', k)
    only_m = {t: m[t] for t in m if t not in c}
    only_c = {t: c[t] for t in c if t not in m}
    diff = {}
    for t in set(m) & set(c):
        if m[t] != c[t]:
            diff[t] = (m[t], c[t])
    if only_m:
        print('only in manifest:', json.dumps(only_m, indent=2)[:2000])
    if only_c:
        print('only in yaml:', json.dumps(only_c, indent=2)[:2000])
    if diff:
        print('diff count', len(diff))
        for t, (mv, cv) in list(diff.items())[:10]:
            print(' ', t)
            print('   manifest:', mv)
            print('   yaml    :', cv)
