#!/usr/bin/env python3
"""Curate the checked-in R1 MJCF from MuJoCo's compiled URDF output."""

from __future__ import annotations

import argparse
import copy
import math
from pathlib import Path
import xml.etree.ElementTree as ET


ACTIVE_JOINTS = (
    "head_yaw_joint",
    "head_pitch_joint",
    "torso_yaw_joint",
    "torso_pitch_joint",
    "knee_pitch_joint",
    "ankle_pitch_joint",
    "left_arm_joint1",
    "left_arm_joint2",
    "left_arm_joint3",
    "left_arm_joint4",
    "left_arm_joint5",
    "left_arm_joint6",
    "left_arm_joint7",
    "right_arm_joint1",
    "right_arm_joint2",
    "right_arm_joint3",
    "right_arm_joint4",
    "right_arm_joint5",
    "right_arm_joint6",
    "right_arm_joint7",
)


def _numbers(text: str | None, count: int, default: tuple[float, ...]) -> tuple[float, ...]:
    if text is None:
        return default
    values = tuple(float(item) for item in text.split())
    if len(values) != count:
        raise RuntimeError(f"expected {count} values, got {text!r}")
    return values


def _quat_from_rpy(rpy: tuple[float, float, float]) -> tuple[float, float, float, float]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    )


def _rotation_from_rpy(
    rpy: tuple[float, float, float],
) -> tuple[tuple[float, float, float], ...]:
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )


def _matmul(
    left: tuple[tuple[float, float, float], ...],
    right: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(
        tuple(sum(left[row][axis] * right[axis][column] for axis in range(3))
              for column in range(3))
        for row in range(3)
    )


def _transpose(
    matrix: tuple[tuple[float, float, float], ...],
) -> tuple[tuple[float, float, float], ...]:
    return tuple(tuple(matrix[column][row] for column in range(3)) for row in range(3))


def _quat_multiply(
    left: tuple[float, float, float, float],
    right: tuple[float, float, float, float],
) -> tuple[float, float, float, float]:
    lw, lx, ly, lz = left
    rw, rx, ry, rz = right
    result = (
        lw * rw - lx * rx - ly * ry - lz * rz,
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
    )
    norm = math.sqrt(sum(value * value for value in result))
    return tuple(value / norm for value in result)


def _quat_rotate(
    quaternion: tuple[float, float, float, float],
    vector: tuple[float, float, float],
) -> tuple[float, float, float]:
    w, x, y, z = quaternion
    rotation = (
        (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
        (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
        (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
    )
    return tuple(sum(rotation[row][axis] * vector[axis] for axis in range(3))
                 for row in range(3))


def _pose_compose(
    first: tuple[tuple[float, float, float], tuple[float, float, float, float]],
    second: tuple[tuple[float, float, float], tuple[float, float, float, float]],
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    first_position, first_quaternion = first
    second_position, second_quaternion = second
    rotated = _quat_rotate(first_quaternion, second_position)
    return (
        tuple(first_position[axis] + rotated[axis] for axis in range(3)),
        _quat_multiply(first_quaternion, second_quaternion),
    )


def _format(values: tuple[float, ...]) -> str:
    return " ".join(f"{value:.17g}" for value in values)


def _origin(element: ET.Element | None) -> tuple[str, str]:
    if element is None:
        return "0 0 0", "1 0 0 0"
    origin = element.find("origin")
    if origin is None:
        return "0 0 0", "1 0 0 0"
    xyz = _numbers(origin.get("xyz"), 3, (0.0, 0.0, 0.0))
    rpy = _numbers(origin.get("rpy"), 3, (0.0, 0.0, 0.0))
    return _format(xyz), _format(_quat_from_rpy(rpy))


def _origin_pose(
    element: ET.Element,
) -> tuple[tuple[float, float, float], tuple[float, float, float, float]]:
    origin = element.find("origin")
    if origin is None:
        return (0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0)
    xyz = _numbers(origin.get("xyz"), 3, (0.0, 0.0, 0.0))
    rpy = _numbers(origin.get("rpy"), 3, (0.0, 0.0, 0.0))
    return xyz, _quat_from_rpy(rpy)


def _restore_kinematic_precision(robot: ET.Element, root: ET.Element) -> None:
    joints_by_name = {joint.get("name"): joint for joint in robot.findall("joint")}
    child_to_joint = {
        joint.find("child").get("link"): joint
        for joint in robot.findall("joint")
        if joint.find("child") is not None
    }
    active_child_links = {
        joints_by_name[name].find("child").get("link") for name in ACTIVE_JOINTS
    }
    body_links = active_child_links | {"base_link"}
    bodies_by_name = {body.get("name"): body for body in root.iter("body")}
    mjcf_joints = {joint.get("name"): joint for joint in root.iter("joint")}

    for joint_name in ACTIVE_JOINTS:
        urdf_joint = joints_by_name[joint_name]
        child_link = urdf_joint.find("child").get("link")
        chain: list[ET.Element] = []
        link = child_link
        while True:
            incoming = child_to_joint.get(link)
            if incoming is None:
                raise RuntimeError(f"cannot find URDF ancestry for {child_link}")
            chain.append(incoming)
            parent_link = incoming.find("parent").get("link")
            if parent_link in body_links:
                break
            link = parent_link

        transform = ((0.0, 0.0, 0.0), (1.0, 0.0, 0.0, 0.0))
        for chain_joint in reversed(chain):
            transform = _pose_compose(transform, _origin_pose(chain_joint))
        body = bodies_by_name.get(child_link)
        if body is None:
            raise RuntimeError(f"compiled MJCF is missing body {child_link}")
        body.set("pos", _format(transform[0]))
        body.set("quat", _format(transform[1]))

        mjcf_joint = mjcf_joints.get(joint_name)
        if mjcf_joint is None:
            raise RuntimeError(f"compiled MJCF is missing joint {joint_name}")
        axis = urdf_joint.find("axis")
        mjcf_joint.set(
            "axis", _format(_numbers(axis.get("xyz") if axis is not None else None,
                                      3, (1.0, 0.0, 0.0)))
        )


def _material_rgba(robot: ET.Element, visual: ET.Element) -> str:
    material = visual.find("material")
    if material is None:
        return "0.9 0.9 0.9 1"
    color = material.find("color")
    if color is not None and color.get("rgba"):
        return color.get("rgba", "0.9 0.9 0.9 1")
    name = material.get("name")
    if name:
        global_material = robot.find(f"material[@name='{name}']/color")
        if global_material is not None and global_material.get("rgba"):
            return global_material.get("rgba", "0.9 0.9 0.9 1")
    return "0.9 0.9 0.9 1"


def _link_visuals(robot: ET.Element) -> dict[str, tuple[str, str, str, str, str | None]]:
    result: dict[str, tuple[str, str, str, str, str | None]] = {}
    for link in robot.findall("link"):
        visual = link.find("visual")
        if visual is None:
            continue
        mesh = visual.find("geometry/mesh")
        if mesh is None or not mesh.get("filename"):
            continue
        filename = Path(mesh.get("filename", "")).name
        position, quaternion = _origin(visual)
        result[link.get("name", "")] = (
            filename,
            position,
            quaternion,
            _material_rgba(robot, visual),
            mesh.get("scale"),
        )
    return result


def _base_inertial(robot: ET.Element) -> ET.Element:
    base = robot.find("link[@name='base_link']/inertial")
    if base is None:
        raise RuntimeError("R1 URDF is missing base_link inertial")
    mass = base.find("mass")
    inertia = base.find("inertia")
    if mass is None or inertia is None:
        raise RuntimeError("R1 base_link inertial is incomplete")
    position, _ = _origin(base)
    origin = base.find("origin")
    rpy = _numbers(origin.get("rpy") if origin is not None else None,
                   3, (0.0, 0.0, 0.0))
    ixx, iyy, izz, ixy, ixz, iyz = tuple(
        float(inertia.get(name, "0"))
        for name in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")
    )
    inertia_in_inertial_frame = (
        (ixx, ixy, ixz),
        (ixy, iyy, iyz),
        (ixz, iyz, izz),
    )
    rotation = _rotation_from_rpy(rpy)
    inertia_in_body_frame = _matmul(
        _matmul(rotation, inertia_in_inertial_frame), _transpose(rotation)
    )
    values = (
        inertia_in_body_frame[0][0],
        inertia_in_body_frame[1][1],
        inertia_in_body_frame[2][2],
        inertia_in_body_frame[0][1],
        inertia_in_body_frame[0][2],
        inertia_in_body_frame[1][2],
    )
    return ET.Element(
        "inertial",
        {
            "pos": position,
            "mass": mass.get("value", "0"),
            "fullinertia": _format(values),
        },
    )


def _joint_efforts(robot: ET.Element) -> dict[str, float]:
    result = {}
    for joint in robot.findall("joint"):
        name = joint.get("name")
        limit = joint.find("limit")
        if name in ACTIVE_JOINTS and limit is not None:
            result[name] = float(limit.get("effort", "0"))
    missing = sorted(set(ACTIVE_JOINTS) - result.keys())
    if missing:
        raise RuntimeError("R1 URDF is missing effort limits: " + ", ".join(missing))
    return result


def curate(urdf_path: Path, compiled_path: Path, output_path: Path) -> None:
    robot = ET.parse(urdf_path).getroot()
    tree = ET.parse(compiled_path)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    asset = root.find("asset")
    if worldbody is None or asset is None:
        raise RuntimeError("compiled MJCF is missing worldbody or asset")

    compiler = root.find("compiler")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("angle", "radian")
    compiler.set("autolimits", "true")
    compiler.set("balanceinertia", "true")

    option = root.find("option")
    if option is None:
        option = ET.Element("option", {"timestep": "0.001", "integrator": "implicitfast"})
        root.insert(1, option)

    existing_world_children = list(worldbody)
    for child in existing_world_children:
        worldbody.remove(child)
    worldbody.append(
        ET.Element(
            "geom",
            {
                "name": "floor",
                "type": "plane",
                "size": "4 4 0.1",
                "rgba": "0.18 0.2 0.24 1",
                "friction": "1 0.01 0.001",
            },
        )
    )
    worldbody.append(
        ET.Element(
            "light",
            {
                "name": "key_light",
                "pos": "0 -3 4",
                "dir": "0 1 -1",
                "diffuse": "0.8 0.8 0.8",
            },
        )
    )
    base_body = ET.SubElement(worldbody, "body", {"name": "base_link"})
    base_body.append(ET.Element("freejoint", {"name": "floating_base"}))
    base_body.append(_base_inertial(robot))
    for child in existing_world_children:
        base_body.append(child)

    allowed = set(ACTIVE_JOINTS)
    for parent in root.iter():
        for child in list(parent):
            if child.tag == "joint" and child.get("name") not in allowed:
                parent.remove(child)

    _restore_kinematic_precision(robot, root)

    visuals = _link_visuals(robot)
    assets_by_name = {element.get("name"): element for element in asset.findall("mesh")}
    bodies_by_name = {element.get("name"): element for element in root.iter("body")}
    visual_assets: dict[str, str] = {}
    for link_name, (filename, _, _, _, scale) in visuals.items():
        visual_asset_name = Path(filename).stem
        if visual_asset_name in assets_by_name:
            visual_assets[link_name] = visual_asset_name
            continue
        attributes = {
            "name": visual_asset_name,
            "content_type": "model/obj",
            "file": f"../meshes/{filename}",
        }
        if scale:
            attributes["scale"] = scale
        asset.append(ET.Element("mesh", attributes))
        assets_by_name[visual_asset_name] = asset[-1]
        visual_assets[link_name] = visual_asset_name

    links_with_visual_geom: set[str] = set()
    for parent in root.iter():
        for geom in list(parent.findall("geom")):
            collision_mesh = geom.get("mesh", "")
            if not collision_mesh.endswith("_collisions"):
                continue
            link_name = collision_mesh[: -len("_collisions")]
            visual_asset_name = visual_assets.get(link_name)
            if not visual_asset_name:
                continue
            geom.set("group", "3")
            geom.set("rgba", "0.2 0.2 0.2 0")
            visual = copy.deepcopy(geom)
            visual.attrib.pop("name", None)
            visual.set("name", f"{link_name}_visual_geom")
            visual.set("mesh", visual_asset_name)
            visual.set("group", "1")
            visual.set("contype", "0")
            visual.set("conaffinity", "0")
            visual.set("rgba", visuals[link_name][3])
            parent.append(visual)
            links_with_visual_geom.add(link_name)

    for link_name, (_, position, quaternion, rgba, _) in visuals.items():
        if link_name in links_with_visual_geom:
            continue
        body = bodies_by_name.get(link_name)
        if body is None:
            continue
        body.append(
            ET.Element(
                "geom",
                {
                    "name": f"{link_name}_visual_geom",
                    "type": "mesh",
                    "mesh": visual_assets[link_name],
                    "pos": position,
                    "quat": quaternion,
                    "group": "1",
                    "contype": "0",
                    "conaffinity": "0",
                    "rgba": rgba,
                },
            )
        )

    for side in ("left", "right"):
        body = bodies_by_name.get(f"{side}_arm_link7")
        if body is None:
            raise RuntimeError(f"compiled MJCF is missing {side}_arm_link7")
        ee_joint = robot.find(f"joint[@name='joint_{side}_arm_ee_link']")
        if ee_joint is None:
            raise RuntimeError(f"R1 URDF is missing {side} EE fixed joint")
        tcp_position, tcp_quaternion = _pose_compose(
            _origin_pose(ee_joint),
            ((0.0, 0.0, 0.1), (1.0, 0.0, 0.0, 0.0)),
        )
        body.append(
            ET.Element(
                "site",
                {
                    "name": f"{side}_tcp_site",
                    "pos": _format(tcp_position),
                    "quat": _format(tcp_quaternion),
                    "size": "0.02",
                    "rgba": "1 0.55 0.05 1" if side == "left" else "0.1 0.55 1 1",
                },
            )
        )
        body.append(
            ET.Element(
                "geom",
                {
                    "name": f"{side}_tcp_handle_geom",
                    "type": "sphere",
                    "pos": _format(tcp_position),
                    "quat": _format(tcp_quaternion),
                    "size": "0.035",
                    "group": "2",
                    "contype": "0",
                    "conaffinity": "0",
                    "rgba": "1 0.55 0.05 0.45" if side == "left" else "0.1 0.55 1 0.45",
                },
            )
        )

    existing_actuator = root.find("actuator")
    if existing_actuator is not None:
        root.remove(existing_actuator)
    actuator = ET.SubElement(root, "actuator")
    efforts = _joint_efforts(robot)
    for joint_name in ACTIVE_JOINTS:
        effort = efforts[joint_name]
        actuator.append(
            ET.Element(
                "motor",
                {
                    "name": f"{joint_name}_motor",
                    "joint": joint_name,
                    "gear": "1",
                    "ctrllimited": "true",
                    "ctrlrange": f"{-effort:.17g} {effort:.17g}",
                },
            )
        )

    ET.indent(tree, space="  ")
    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree.write(output_path, encoding="utf-8", xml_declaration=True)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--urdf", type=Path, required=True)
    parser.add_argument("--compiled-mjcf", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    curate(args.urdf, args.compiled_mjcf, args.output)


if __name__ == "__main__":
    main()
