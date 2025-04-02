#!/usr/bin/env python3
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

# Description: Script to add license headers to all Python files in the project.

import os
import sys
import re

# The license header text
LICENSE_HEADER = '''"""
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

'''

# Directories to skip
SKIP_DIRS = ['.git', '.vscode', '__pycache__', 'venv', 'env', '.env', 'build', 'dist']

# Check if the file already has a license header
def has_license(content):
    return 'MIT License' in content[:500] and 'Copyright' in content[:500]

def process_file(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Skip if file already has a license header
    if has_license(content):
        print(f"Skipping {file_path} - already has license header")
        return False
    
    # Handle shebang line if present
    if content.startswith('#!'):
        shebang_end = content.find('\n') + 1
        new_content = content[:shebang_end] + '\n' + LICENSE_HEADER + content[shebang_end:]
    else:
        new_content = LICENSE_HEADER + content
    
    with open(file_path, 'w', encoding='utf-8') as f:
        f.write(new_content)
    
    print(f"Added license header to {file_path}")
    return True

def process_directory(directory):
    files_processed = 0
    
    for root, dirs, files in os.walk(directory):
        # Skip directories in SKIP_DIRS
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        
        for file in files:
            if file.endswith('.py') and file != 'add_license_headers.py':
                file_path = os.path.join(root, file)
                if process_file(file_path):
                    files_processed += 1
    
    return files_processed

if __name__ == '__main__':
    # Get the directory to process, default to current directory
    directory = sys.argv[1] if len(sys.argv) > 1 else '.'
    
    if not os.path.isdir(directory):
        print(f"Error: {directory} is not a valid directory")
        sys.exit(1)
    
    print(f"Adding license headers to Python files in {directory}")
    count = process_directory(directory)
    print(f"Added license headers to {count} files") 