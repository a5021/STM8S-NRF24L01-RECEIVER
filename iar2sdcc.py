#!/usr/bin/env python3
"""
iar2sdcc.py — Patch IAR-specific constructs for SDCC compatibility

Transforms multi-bit SFR field assignments (which SDCC cannot express
via __sbit) into full-register read-modify-write operations.

Usage:
    python3 iar2sdcc.py <input.c> [output.c]
    If output.c is omitted, the file is modified in place.
"""
import re, sys

def patch_source(src: str) -> str:
    # SPI_CR1_BR = N  ->  SPI_CR1 = (SPI_CR1 & ~0x0E) | (N << 1)
    src = re.sub(
        r'SPI_CR1_BR\s*=\s*(\d+)\s*;',
        r'SPI_CR1 = (SPI_CR1 & ~0x0E) | (\1 << 1);',
        src
    )
    return src

def main():
    if len(sys.argv) < 2:
        print("Usage: iar2sdcc.py <input.c> [output.c]")
        sys.exit(1)
    src_path = sys.argv[1]
    with open(src_path, 'r', encoding='latin-1') as f:
        src = f.read()
    patched = patch_source(src)
    dst_path = sys.argv[2] if len(sys.argv) > 2 else src_path
    with open(dst_path, 'w', encoding='utf-8') as f:
        f.write(patched)

if __name__ == '__main__':
    main()