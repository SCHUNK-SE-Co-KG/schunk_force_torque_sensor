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

## Run tests with the FTS dummy
1. Navigate into the `schunk_fts_dummy` repo and start the dummy with
    ```bash
    cargo run
    ```

2. Set environment variables in the terminals in which you test
    ```bash
    export FTS_HOST="127.0.0.1"
    export FTS_PORT=8082
    ```
    The tests will then connect to the FTS dummy.
