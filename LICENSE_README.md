# Source Code Licensing

This project is licensed under the MIT License. To properly maintain licensing information across the codebase, we provide utilities for applying license headers to all Python source files.

## License Files

- `LICENSE` - The main MIT license file for the project
- `LICENSE_HEADER.py` - Example license header for Python files
- `add_license_headers.py` - Utility script to automatically add license headers to Python files

## Adding License Headers to Source Files

You can add the MIT license header to all Python files in your project by running:

```bash
python add_license_headers.py
```

This will recursively search through the project directory and add the license header to any Python file that doesn't already have one.

You can also specify a specific directory to process:

```bash
python add_license_headers.py path/to/directory
```

## License Header Format

Each Python source file should have the following license header at the top:

```python
"""
MIT License

Copyright (c) 2023-2025 omni-mcp

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
"""
```

Followed by a brief description of the file's purpose:

```python
# Description: This file implements...
```

## Adding License Headers Manually

If you prefer to add headers manually, or are creating a new file, simply copy the header from `LICENSE_HEADER.py` and paste it at the beginning of your Python file.

## Excluded Directories

The automated script skips the following directories:
- `.git`
- `.vscode`
- `__pycache__`
- `venv`, `env`, `.env`
- `build`
- `dist`

If you need to modify this list, edit the `SKIP_DIRS` variable in `add_license_headers.py`. 