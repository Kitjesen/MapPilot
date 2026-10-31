"""Author a bounded MuJoCo prop test scene; parameters are design assumptions."""

from __future__ import annotations

import argparse
import math
import xml.etree.ElementTree as ET
from pathlib import Path


def add(parent, tag, **attrs):
    return ET.SubElement(parent, tag, {key: str(value) for key, value in attrs.items()})


def body(parent, name, pos):
    return add(parent, "body", name=name, pos=pos)


def box(parent, name, size, pos="0 0 0", mass=1, color=".6 .65 .7 1"):
    return add(parent, "geom", name=name, type="box", size=size, pos=pos, mass=mass, rgba=color)


def hinge(parent, name, axis, limits=None, damping=1):
    args = dict(name=name, type="hinge", axis=axis, damping=damping)
    if limits:
        args.update(limited="true", range=limits)
    return add(parent, "joint", **args)


def build_scene():
    root = ET.Element("mujoco", model="industrial_interaction_lab")
    add(root, "compiler", angle="radian", autolimits="true")
    add(root, "option", timestep=".002", integrator="implicitfast", gravity="0 0 -9.81")
    default = add(root, "default")
    add(default, "geom", friction="1.0 .01 .001", condim="4", solref=".01 1")
    world = add(root, "worldbody")
    add(world, "geom", name="floor", type="plane", size="30 30 .1", rgba=".55 .58 .6 1")
    actuators = add(root, "actuator")

    # Free chassis and tire contacts, not a translated animation.
    truck = body(world, "truck/chassis", "0 0 .47")
    add(truck, "freejoint", name="truck/free")
    box(truck, "truck/chassis_collision", "3.1 1 .6", "-.2 0 1.1", 3500)
    for side, y in [("l", 1.05), ("r", -1.05)]:
        for axle, x in [("front", 2.4), ("rear", -1.76)]:
            parent = truck
            pos = f"{x} {y} 0"
            if axle == "front":
                parent = body(truck, f"truck/steer_{side}", pos)
                pos = "0 0 0"
                hinge(parent, f"truck/steer_{side}", "0 0 1", "-.5 .5", 8)
                add(parent, "inertial", mass=".5", diaginertia=".001 .001 .001", pos="0 0 0")
                add(
                    actuators,
                    "position",
                    name=f"truck/steer_{side}",
                    joint=f"truck/steer_{side}",
                    kp="600",
                    kv="60",
                    ctrlrange="-.5 .5",
                    forcerange="-300 300",
                )
            wheel = body(parent, f"truck/wheel_{axle}_{side}", pos)
            hinge(wheel, f"truck/spin_{axle}_{side}", "0 1 0", damping=0.2)
            add(
                wheel,
                "geom",
                name=f"truck/tire_{axle}_{side}",
                type="cylinder",
                size=".45 .12",
                quat=".70710678 .70710678 0 0",
                mass="35",
                rgba=".06 .07 .08 1",
            )
            if axle == "rear":
                add(
                    actuators,
                    "velocity",
                    name=f"truck/drive_{side}",
                    joint=f"truck/spin_{axle}_{side}",
                    kv="400",
                    ctrlrange="-8 8",
                    forcerange="-1800 1800",
                )

    # A pushable bin with passive wheel joints and a hinged lid.
    bin_root = body(world, "bin/chassis", "0 -6 .19")
    add(bin_root, "freejoint", name="bin/free")
    box(bin_root, "bin/shell", ".6 .5 .46", "0 0 .66", 90, ".5 .55 .57 1")
    lid = body(bin_root, "bin/lid", "-.6 0 1.14")
    hinge(lid, "bin/lid", "0 -1 0", "0 1.9", 2)
    box(lid, "bin/lid_collision", ".6 .51 .035", ".6 0 0", 5, ".06 .18 .12 1")
    for i, (x, y) in enumerate([(-0.45, -0.43), (-0.45, 0.43), (0.45, -0.43), (0.45, 0.43)]):
        caster = body(bin_root, f"bin/caster_{i}", f"{x} {y} 0")
        hinge(caster, f"bin/swivel_{i}", "0 0 1", damping=0.05)
        add(caster, "inertial", mass=".4", diaginertia=".003 .003 .003", pos="0 0 .03")
        wheel = body(caster, f"bin/wheel_{i}", ".035 0 0")
        hinge(wheel, f"bin/roll_{i}", "0 1 0", damping=0.02)
        add(
            wheel,
            "geom",
            name=f"bin/tire_{i}",
            type="cylinder",
            size=".18 .045",
            quat=".70710678 .70710678 0 0",
            mass="1",
            rgba=".07 .08 .09 1",
        )
    add(actuators, "motor", name="bin/lid_torque", joint="bin/lid", ctrlrange="-60 60")

    cabinet = body(world, "cabinet/frame", "5 -5 .08")
    box(cabinet, "cabinet/back", ".03 .57 .9", "-.28 0 .9", 35)
    for side, sign in [("left", 1), ("right", -1)]:
        box(cabinet, f"cabinet/side_{side}", ".28 .025 .9", f"0 {sign * 0.565} .9", 8)
        door = body(cabinet, f"cabinet/door_{side}", f".28 {sign * 0.55} .9")
        hinge(door, f"cabinet/hinge_{side}", f"0 0 {sign}", "0 1.919862", 2)
        box(door, f"cabinet/panel_{side}", ".025 .265 .87", f"0 {-sign * 0.27} 0", 12, ".78 .8 .78 1")
        add(actuators, "motor", name=f"cabinet/torque_{side}", joint=f"cabinet/hinge_{side}", ctrlrange="-35 35")

    hvac = body(world, "hvac/base", "-6 5 0")
    box(hvac, "hvac/casing", "1.1 .48 .6", "0 0 .6", 180)
    for i, x in enumerate([-0.55, 0.55]):
        rotor = body(hvac, f"hvac/rotor_{i}", f"{x} 0 1.25")
        hinge(rotor, f"hvac/fan_{i}", "0 0 1", damping=0.02)
        box(rotor, f"hvac/blade_a_{i}", ".36 .045 .012", mass=0.3, color=".1 .12 .13 1")
        box(rotor, f"hvac/blade_b_{i}", ".045 .36 .012", mass=0.3, color=".1 .12 .13 1")
        add(
            actuators,
            "velocity",
            name=f"hvac/fan_drive_{i}",
            joint=f"hvac/fan_{i}",
            kv=".05",
            ctrlrange="0 60",
            forcerange="-2 2",
        )

    valve = body(world, "valve/base", "5 5 .7")
    add(
        valve,
        "geom",
        name="valve/pipe",
        type="capsule",
        fromto="-.7 0 0 .7 0 0",
        size=".13",
        mass="40",
        rgba=".05 .17 .32 1",
    )
    hand = body(valve, "valve/handwheel", "0 0 .55")
    hinge(hand, "valve/turn", "0 0 1", f"0 {12 * math.pi}", 0.15)
    add(hand, "joint", name="valve/spindle_lift", type="slide", axis="0 0 1", range="0 .12", damping="4")
    box(hand, "valve/spoke_a", ".25 .025 .02", mass=0.4, color=".7 .05 .03 1")
    box(hand, "valve/spoke_b", ".025 .25 .02", mass=0.4, color=".7 .05 .03 1")
    equality = add(root, "equality")
    add(
        equality,
        "joint",
        joint1="valve/spindle_lift",
        joint2="valve/turn",
        polycoef=f"0 {0.12 / (12 * math.pi)} 0 0 0",
        solref=".004 1",
        solimp=".999 .999 .001",
    )
    add(actuators, "motor", name="valve/torque", joint="valve/turn", ctrlrange="-3 3")

    hydrant = body(world, "hydrant/base", "8 -4 0")
    add(hydrant, "geom", name="hydrant/barrel", type="cylinder", size=".16 .48", pos="0 0 .48", rgba=".75 .03 .02 1")
    nut = body(hydrant, "hydrant/nut", "0 0 1.0")
    hinge(nut, "hydrant/turn", "0 0 1", "0 31.4159", 0.2)
    box(nut, "hydrant/operating_nut", ".045 .045 .04", mass=0.4, color=".2 .22 .24 1")
    add(actuators, "motor", name="hydrant/torque", joint="hydrant/turn", ctrlrange="-3 3")

    barrier = body(world, "barrier/base", "-5 -6 0")
    box(barrier, "barrier/cabinet", ".2 .22 .55", "0 0 .55", 60, ".85 .65 .08 1")
    boom = body(barrier, "barrier/boom", "0 -.3 1.05")
    hinge(boom, "barrier/lift", "0 -1 0", "0 1.570796", 8)
    box(boom, "barrier/arm", "1.5 .045 .055", "1.5 0 0", 9, ".9 .9 .85 1")
    add(
        actuators,
        "position",
        name="barrier/open",
        joint="barrier/lift",
        kp="500",
        kv="50",
        ctrlrange="0 1.570796",
        forcerange="-350 350",
    )

    frame = body(world, "service_door/frame", "-7 0 0")
    for side, sign in [("left", 1), ("right", -1)]:
        box(frame, f"service_door/post_{side}", ".09 .055 1.05", f"0 {sign * 1.05} 1.05", 20)
        leaf = body(frame, f"service_door/{side}", f".13 {sign} 1.0")
        hinge(leaf, f"service_door/hinge_{side}", f"0 0 {sign}", "0 1.919862", 4)
        box(leaf, f"service_door/panel_{side}", ".035 .49 .98", f"0 {-sign * 0.5} 0", 25, ".52 .59 .61 1")
        add(
            actuators,
            "motor",
            name=f"service_door/torque_{side}",
            joint=f"service_door/hinge_{side}",
            ctrlrange="-50 50",
        )
    box(frame, "service_door/header", ".09 1.1 .055", "0 0 2.1", 15)

    forklift = body(world, "forklift/chassis", "0 7 .34")
    add(forklift, "freejoint", name="forklift/free")
    box(forklift, "forklift/body", ".85 .55 .35", "0 0 .5", 2500, ".8 .55 .03 1")
    for i, (x, y) in enumerate([(-0.6, -0.6), (-0.6, 0.6), (0.65, -0.6), (0.65, 0.6)]):
        wheel = body(forklift, f"forklift/wheel_{i}", f"{x} {y} 0")
        hinge(wheel, f"forklift/roll_{i}", "0 1 0", damping=0.1)
        add(
            wheel,
            "geom",
            name=f"forklift/tire_{i}",
            type="cylinder",
            size=".32 .09",
            quat=".70710678 .70710678 0 0",
            mass="15",
            rgba=".05 .06 .07 1",
        )
    mast = body(forklift, "forklift/mast", "1 0 .2")
    hinge(mast, "forklift/tilt", "0 1 0", "-.12 .12", 20)
    box(mast, "forklift/mast_collision", ".07 .45 1.1", "0 0 1.1", 80, ".1 .12 .13 1")
    carriage = body(mast, "forklift/carriage", ".18 0 .15")
    add(carriage, "joint", name="forklift/lift", type="slide", axis="0 0 1", range="0 1.8", damping="50")
    for i, y in enumerate([-0.3, 0.3]):
        box(carriage, f"forklift/fork_{i}", ".6 .06 .035", f".6 {y} 0", 12, ".15 .16 .17 1")
    add(
        actuators,
        "position",
        name="forklift/lift",
        joint="forklift/lift",
        kp="6000",
        kv="300",
        ctrlrange="0 1.8",
        forcerange="-12000 12000",
    )
    add(
        actuators,
        "position",
        name="forklift/tilt",
        joint="forklift/tilt",
        kp="20000",
        kv="500",
        ctrlrange="-.12 .12",
        forcerange="-6000 6000",
    )

    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode") + "\n"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(build_scene(), encoding="utf-8")


if __name__ == "__main__":
    main()
