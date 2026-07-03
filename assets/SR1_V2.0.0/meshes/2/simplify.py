#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import open3d as o3d
import trimesh


# 原始mesh目录
MESH_DIR = "./"

# 输出目录
COLLISION_DIR = os.path.join(MESH_DIR, "collision")

# 保留比例
REDUCTION_RATIO = 0.1

# 每个link最少面数
MIN_TRIANGLES = 200

# 每个link最多面数
MAX_TRIANGLES = 2500


def simplify_stl(input_file, output_file):

    try:

        mesh = o3d.io.read_triangle_mesh(input_file)

        if len(mesh.triangles) == 0:
            print(f"[SKIP] Empty mesh: {input_file}")
            return False

        original_triangles = len(mesh.triangles)

        target_triangles = int(
            original_triangles * REDUCTION_RATIO
        )

        target_triangles = max(
            MIN_TRIANGLES,
            target_triangles
        )

        target_triangles = min(
            MAX_TRIANGLES,
            target_triangles
        )

        if target_triangles >= original_triangles:

            vertices = mesh.vertices
            faces = mesh.triangles

            tm = trimesh.Trimesh(
                vertices=vertices,
                faces=faces,
                process=False
            )

            tm.export(output_file)

            print(
                f"[COPY] {os.path.basename(input_file)} "
                f"{original_triangles} triangles"
            )

            return True

        simplified = mesh.simplify_quadric_decimation(
            target_number_of_triangles=target_triangles
        )

        simplified.remove_duplicated_vertices()
        simplified.remove_degenerate_triangles()
        simplified.remove_duplicated_triangles()
        simplified.remove_non_manifold_edges()

        vertices = simplified.vertices
        faces = simplified.triangles

        tm = trimesh.Trimesh(
            vertices=vertices,
            faces=faces,
            process=False
        )

        tm.export(output_file)

        print(
            f"[OK] {os.path.basename(input_file)} : "
            f"{original_triangles} -> "
            f"{len(tm.faces)} triangles"
        )

        return True

    except Exception as e:

        print(f"[ERROR] {input_file}")
        print(e)

        return False


def main():

    os.makedirs(
        COLLISION_DIR,
        exist_ok=True
    )

    stl_files = sorted([
        f for f in os.listdir(MESH_DIR)
        if f.lower().endswith(".stl")
    ])

    print(f"Found {len(stl_files)} STL files\n")

    success_cnt = 0

    for stl_file in stl_files:

        input_path = os.path.join(
            MESH_DIR,
            stl_file
        )

        output_path = os.path.join(
            COLLISION_DIR,
            stl_file
        )

        if simplify_stl(
            input_path,
            output_path
        ):
            success_cnt += 1

    print("\n----------------------------------")
    print(
        f"Finished: {success_cnt}/{len(stl_files)} files"
    )
    print(
        f"Output dir: {COLLISION_DIR}"
    )
    print("----------------------------------")


if __name__ == "__main__":
    main()