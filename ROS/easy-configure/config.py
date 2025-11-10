#!/usr/bin/env python3

import yaml
import argparse
import sys
from pathlib import Path

# --- Helper functions (no changes here) ---
TYPE_MAP = { bool: "bool", int: "int", float: "double", str: "std::string" }

def to_title_case(snake_str):
    components = snake_str.replace('-', '_').split('_')
    return "".join(x.title() for x in components)

def get_cpp_type(value, key_name):
    value_type = type(value)
    if value_type in TYPE_MAP: return TYPE_MAP[value_type]
    if value_type is list:
        if not value: raise TypeError(f"Error in key '{key_name}': YAML list is empty.")
        element_type = get_cpp_type(value[0], f"{key_name}[0]")
        return f"std::vector<{element_type}>"
    if value_type is dict:
        return value["$type"] if "$type" in value else f"{to_title_case(key_name)}Config"
    raise TypeError(f"Unsupported data type '{value_type.__name__}' for key '{key_name}'.")

def generate_structs_recursive(data, defined_types):
    new_struct_definitions = []
    data_items = {k: v for k, v in data.items() if k != "$type"}
    for key, value in data_items.items():
        if isinstance(value, dict):
            struct_name = get_cpp_type(value, key)
            if struct_name not in defined_types:
                defined_types.add(struct_name)
                member_declarations = []
                value_items = {k: v for k, v in value.items() if k != "$type"}
                for member_key, member_value in value_items.items():
                    cpp_type = get_cpp_type(member_value, member_key)
                    member_declarations.append(f"    {cpp_type} {member_key};")
                nested_defs = generate_structs_recursive(value, defined_types)
                new_struct_definitions.extend(nested_defs)
                members_code = "\n".join(member_declarations)
                new_struct_definitions.append(f"struct {struct_name} {{\n{members_code}\n}};")
    return new_struct_definitions

def generate_loading_functions_recursive(data, defined_loaders):
    new_loader_definitions = []
    data_items = {k: v for k, v in data.items() if k != "$type"}
    for key, value in data_items.items():
        if isinstance(value, dict):
            struct_type_name = get_cpp_type(value, key)
            if struct_type_name not in defined_loaders:
                defined_loaders.add(struct_type_name)
                nested_loaders = generate_loading_functions_recursive(value, defined_loaders)
                new_loader_definitions.extend(nested_loaders)
                loader_func_name = f"load_{struct_type_name}"
                func_body = []
                value_items = {k: v for k, v in value.items() if k != "$type"}
                for member_key, member_value in value_items.items():
                    if isinstance(member_value, dict):
                        nested_struct_type = get_cpp_type(member_value, member_key)
                        nested_loader_name = f"load_{nested_struct_type}"
                        func_body.append(f"    {nested_loader_name}(ros::NodeHandle(nh, \"{member_key}\"), config.{member_key});")
                    else:
                        func_body.append(f"    nh.getParam(\"{member_key}\", config.{member_key});")
                body_code = "\n".join(func_body)
                func_signature = f"inline void {loader_func_name}(const ros::NodeHandle& nh, {struct_type_name}& config)"
                new_loader_definitions.append(f"{func_signature} {{\n{body_code}\n}}")
    return new_loader_definitions

def generate_update_helpers_recursive(data, defined_helpers):
    new_helper_definitions = []
    data_items = {k: v for k, v in data.items() if k != "$type"}
    for key, value in data_items.items():
        if isinstance(value, dict):
            struct_type_name = get_cpp_type(value, key)
            if struct_type_name not in defined_helpers:
                defined_helpers.add(struct_type_name)
                nested_helpers = generate_update_helpers_recursive(value, defined_helpers)
                new_helper_definitions.extend(nested_helpers)
                value_items = {k: v for k, v in value.items() if k != "$type"}
                for member_key, member_value in value_items.items():
                    if not isinstance(member_value, dict):
                        # <<<<< THIS IS ONE OF THE CORRECTED LINES >>>>>
                        specialization = (f"template<> inline const char* get_param_name<&{struct_type_name}::{member_key}>()"
                                          f" {{ return \"{member_key}\"; }}")
                        new_helper_definitions.append(specialization)
    return new_helper_definitions

def main():
    parser = argparse.ArgumentParser(description="Generate C++ header with structs and loaders from YAML.")
    parser.add_argument("-o", "--output", required=True, help="Path to the output C++ header file.")
    parser.add_argument("-n", "--name", required=True, help="Name of the root C++ struct.")
    args = parser.parse_args()

    try:
        script_path = Path(__file__).resolve(); config_dir = script_path.parent
        input_yaml_path = config_dir / "config.yaml"
        with open(input_yaml_path, 'r') as f: yaml_data = yaml.safe_load(f)

        # Generate all code components
        defined_structs = set()
        all_struct_defs = generate_structs_recursive(yaml_data, defined_structs)
        main_struct_members = [f"    {get_cpp_type(v, k)} {k};" for k, v in yaml_data.items()]
        main_struct_code = f"struct {args.name} {{\n" + "\n".join(main_struct_members) + "\n};" # Corrected bracket

        defined_loaders = set()
        all_loader_defs = generate_loading_functions_recursive(yaml_data, defined_loaders)
        main_loader_body = []
        for key, value in yaml_data.items():
            if isinstance(value, dict):
                main_loader_body.append(f"    load_{get_cpp_type(value, key)}(ros::NodeHandle(nh, \"{key}\"), config.{key});")
            else:
                main_loader_body.append(f"    nh.getParam(\"{key}\", config.{key});")
        main_loader_code = f"inline void load_from_param_server(const ros::NodeHandle& nh, {args.name}& config) {{\n" + "\n".join(main_loader_body) + "\n}" # Corrected bracket

        defined_helpers = set()
        all_update_helpers = generate_update_helpers_recursive(yaml_data, defined_helpers)
        for key, value in yaml_data.items():
             if not isinstance(value, dict):
                # <<<<< THIS IS THE OTHER CORRECTED LINE >>>>>
                specialization = (f"template<> inline const char* get_param_name<&{args.name}::{key}>()"
                                  f" {{ return \"{key}\"; }}")
                all_update_helpers.append(specialization)

        # Write all components to the output file
        output_path = Path(args.output); output_path.parent.mkdir(parents=True, exist_ok=True)
        with open(output_path, 'w') as f:
            package_name = args.name.replace('Config', '')
            f.write(f"// AUTOGENERATED FILE for package {package_name}. DO NOT EDIT.\n\n")
            f.write("#pragma once\n\n#include <ros/ros.h>\n#include <string>\n#include <vector>\n\n")
            f.write(f"namespace {package_name}_config {{\n\n")
            if all_struct_defs: f.write("\n".join(all_struct_defs) + "\n\n")
            f.write(main_struct_code + "\n\n")
            if all_loader_defs: f.write("\n".join(all_loader_defs) + "\n\n")
            f.write(main_loader_code + "\n\n")
            f.write("// --- Dynamic Update Helpers ---\n")
            f.write("template<auto MemberPtr> const char* get_param_name() = delete;\n\n")
            if all_update_helpers: f.write("\n".join(all_update_helpers) + "\n\n")
            f.write(
                "template<class Struct, auto MemberPtr>\n"
                "bool update_param(const ros::NodeHandle& nh, Struct& config) {\n"
                "    return nh.getParam(get_param_name<MemberPtr>(), config.*MemberPtr);\n"
                "}\n"
            )
            f.write(f"\n}} // namespace {package_name}_config\n")

        print(f"Successfully generated C++ header with structs, loaders, and update helpers at {args.output}")
    except Exception as e:
        print(f"Error: {e}", file=sys.stderr)
        sys.exit(1)

if __name__ == "__main__":
    main()