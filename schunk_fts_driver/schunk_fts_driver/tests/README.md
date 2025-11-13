
## Run tests with the FTS dummy
1. Navigate into the `schunk_fts_dummy` repo and start the dummy with
    ```bash
    cargo run
    ```

2. The IP of the dummy is set in the [fixtures.py](../../../schunk_fts_library/schunk_fts_library/fixtures.py) of schunk_fts_library.
    The Dummy will automatically be used if no real sensor is connected. But the dummy must be running before starting the tests.

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
