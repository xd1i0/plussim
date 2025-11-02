# Concurrent builds

For MacOS 
```bash
$ make -j$(sysctl -n hw.ncpu)
```

For Linux 
```bash
$ make -j$(nproc)
```
