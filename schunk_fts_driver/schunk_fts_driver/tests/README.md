
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

## Test coverage
```bash
pip install --user pytest coverage
```

```bash
coverage run -m pytest tests/
coverage report
```

## Run individual tests
You can test individual functions, e.g. with
```bash
pytest test_driver::<function-name>
```

Especially the ROS2-related functions may block eternally due to subsequent fails.
Unfortunately, hitting `ctrl-C` will discard useful output for this case.

To catch the first failure message, use
```bash
pytest --maxfail=1 <some-test>
```
