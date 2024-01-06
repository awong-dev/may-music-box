Setup instrauctions

```bash
export ESP_ADF=<adf directory>
cd <idf directory>  # Make sure is ESP-IDF 5 or higher.
. ./export.sh
```

This will setup a virtual env in `~/.espressif/python_env`

Reset build
```bash
idf.py fullclean
idf.py build
```
