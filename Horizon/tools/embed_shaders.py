#!/usr/bin/env python3
"""Embed compiled SPIR-V blobs into a C translation unit."""
import os
import struct
import sys

out_dir = sys.argv[1]
spv_files = sys.argv[2:]

entries = []
arrays = []

for idx, path in enumerate(spv_files):
    name = os.path.basename(path)
    with open(path, "rb") as f:
        data = f.read()
    if len(data) % 4:
        data += b"\x00" * (4 - len(data) % 4)
    words = struct.unpack("<%dI" % (len(data) // 4), data)
    arr_name = "hr_spv_%d" % idx
    body = ", ".join("0x%08xu" % w for w in words)
    arrays.append("static const uint32_t %s[] = {%s};" % (arr_name, body))
    entries.append('    { "%s", %s, %d }' % (name, arr_name, len(words)))

with open(os.path.join(out_dir, "shaders_embedded.h"), "w") as f:
    f.write("""#ifndef HR_SHADERS_EMBEDDED_H
#define HR_SHADERS_EMBEDDED_H
#include <stdint.h>
typedef struct HrShaderBlob {
    const char* name;
    const uint32_t* words;
    uint32_t word_count;
} HrShaderBlob;
extern const HrShaderBlob hr_shader_blobs[];
extern const int hr_shader_blob_count;
#endif
""")

with open(os.path.join(out_dir, "shaders_embedded.c"), "w") as f:
    f.write('#include "shaders_embedded.h"\n\n')
    f.write("\n".join(arrays))
    f.write("\n\nconst HrShaderBlob hr_shader_blobs[] = {\n")
    f.write(",\n".join(entries))
    f.write("\n};\nconst int hr_shader_blob_count = %d;\n" % len(entries))

print("embedded %d shader blobs" % len(entries))
