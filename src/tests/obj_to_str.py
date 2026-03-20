import sys

def main():
    if len(sys.argv) != 3:
        print("usage: python obj_to_c_string.py input.obj output.h")
        sys.exit(1)

    in_path = sys.argv[1]
    out_path = sys.argv[2]

    with open(in_path, "r", encoding="utf-8") as f:
        lines = f.readlines()

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("static const char test_obj[] =\n")
        for line in lines:
            line = line.rstrip("\n")
            line = line.replace("\\", "\\\\").replace('"', '\\"')
            f.write(f"\"{line}\\n\"\n")
        f.write(";\n")

if __name__ == "__main__":
    main()