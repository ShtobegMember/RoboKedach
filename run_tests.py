"""
run_tests.py - Test Suite Runner for RoboKedach
Automatically configures the environment paths and executes the pytest suite.
"""

import sys
import os
import pytest

def main():
    # 1. Add the project root to the Python path
    # This ensures that imports like 'from core.config_loader import CONFIG' work 
    # regardless of how this script is executed.
    project_root = os.path.abspath(os.path.dirname(__file__))
    if project_root not in sys.path:
        sys.path.insert(0, project_root)

    print("🚀 Starting RoboKedach Test Suite...")
    print(f"📁 Project Root: {project_root}")
    print("-" * 50)

    # 2. Define the pytest arguments
    # -v: Verbose output (lists each test individually)
    # -s: Disables output capturing (allows print() statements to show in the console)
    # "tests/": The directory where your test files are located
    pytest_args = [
        "-v", 
        "-s", 
        "tests/"
    ]

    # You can pass additional arguments from the command line through to pytest
    # Example: python run_tests.py -k test_network
    if len(sys.argv) > 1:
        pytest_args.extend(sys.argv[1:])

    # 3. Execute the tests
    exit_code = pytest.main(pytest_args)

    # 4. Handle the result
    print("-" * 50)
    if exit_code == pytest.ExitCode.OK:
        print("✅ All tests completed successfully!")
    elif exit_code == pytest.ExitCode.NO_TESTS_COLLECTED:
        print("⚠️  No tests were found. Make sure your test files are in the 'tests/' folder and named 'test_*.py'.")
    else:
        print(f"❌ Test run failed (Exit Code: {exit_code}).")

    sys.exit(exit_code)

if __name__ == "__main__":
    main()