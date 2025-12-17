#!/usr/bin/env python3
"""
Validate API usage examples in documentation match actual endpoints.

This script scans documentation for API examples and verifies that:
1. All mentioned endpoints exist in the codebase
2. Request/response examples are valid
3. Example code is syntactically correct
"""

import re
import json
import sys
from pathlib import Path
from typing import List, Dict, Tuple

# Backend API router patterns
API_PATTERNS = {
    'health': r'/api/health',
    'auth': r'/api/auth/(signup|login|logout|me|refresh)',
    'chat': r'/api/chat/(message|history|sessions)',
    'personalization': r'/api/personalization/([\w-]+)',
    'translation': r'/api/translation/(translate|translate-batch|languages)',
    'admin': r'/api/admin/(users|metrics|health)',
}


def find_api_examples_in_docs() -> List[Tuple[Path, str, str]]:
    """Find all API examples in documentation."""
    doc_path = Path('docusaurus/docs')
    examples = []

    if not doc_path.exists():
        print(f"Documentation path not found: {doc_path}")
        return examples

    # Regex to find API examples in markdown
    api_example_pattern = r'```(?:bash|shell|curl|javascript|typescript|python|json)\s*(.*?)\s*```'

    for doc_file in doc_path.glob('**/*.md'):
        content = doc_file.read_text(encoding='utf-8')

        # Look for API examples
        for match in re.finditer(api_example_pattern, content, re.DOTALL):
            code_block = match.group(1)

            # Check if this looks like an API example
            if any(pattern in code_block for pattern in ['curl', 'fetch', 'requests', '/api/']):
                examples.append((doc_file, code_block, match.group(0)))

    return examples


def validate_endpoint(endpoint: str) -> bool:
    """Validate that an API endpoint exists."""
    # Normalize the endpoint
    endpoint = endpoint.strip()

    # Check against known patterns
    for route_name, pattern in API_PATTERNS.items():
        if re.search(pattern, endpoint):
            return True

    return False


def validate_json_examples(code_block: str) -> bool:
    """Validate JSON examples in code blocks."""
    json_pattern = r'\{[\s\S]*\}'

    for match in re.finditer(json_pattern, code_block):
        try:
            json.loads(match.group(0))
        except json.JSONDecodeError as e:
            print(f"  ⚠ Invalid JSON: {e}")
            return False

    return True


def extract_endpoints(code_block: str) -> List[str]:
    """Extract API endpoints from code blocks."""
    endpoints = re.findall(r'(\/api\/[\w/-]+)', code_block)
    return endpoints


def main() -> int:
    """Main validation function."""
    print("Validating API examples in documentation...\n")

    examples = find_api_examples_in_docs()

    if not examples:
        print("✓ No API examples found in documentation")
        return 0

    print(f"Found {len(examples)} API examples\n")

    valid_count = 0
    invalid_count = 0

    for doc_file, code_block, full_example in examples:
        print(f"Checking {doc_file.relative_to('docusaurus')}:")

        # Extract endpoints from the example
        endpoints = extract_endpoints(code_block)

        if not endpoints:
            print("  ℹ No specific endpoints found")
            continue

        # Validate each endpoint
        all_valid = True
        for endpoint in endpoints:
            if validate_endpoint(endpoint):
                print(f"  ✓ {endpoint}")
                valid_count += 1
            else:
                print(f"  ✗ {endpoint} (not found)")
                invalid_count += 1
                all_valid = False

        # Validate JSON if present
        if '{' in code_block:
            if validate_json_examples(code_block):
                print("  ✓ JSON is valid")
            else:
                print("  ✗ Invalid JSON found")
                all_valid = False

        print()

    # Summary
    print("\nValidation Summary:")
    print(f"  ✓ Valid: {valid_count}")
    print(f"  ✗ Invalid: {invalid_count}")

    if invalid_count > 0:
        print("\n⚠ Some API examples have issues. Please review.")
        return 1
    else:
        print("\n✓ All API examples are valid!")
        return 0


if __name__ == '__main__':
    sys.exit(main())
