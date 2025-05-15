# Run tests locally

Inside the dummy's package, install test-related dependencies with

```bash
pip install --user pytest coverage
```

You can run the tests either directly in this folder with `pytest .` or with more output through

```bash
coverage run -m pytest .
coverage report  # for console output
coverage html    # for web-based output
```
