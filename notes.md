# Concurrent builds

For MacOS 
```bash
$ make -j$(sysctl -n hw.ncpu)
```

For Linux 
```bash
$ make -j$(nproc)
```

# Testing
enable_testing() # can be removed when you dont want to have tests enabled

The setup is currently so that the visual part is in the **iris** project. The calculation is in
**stellaris**. Stellaris is the library that does the heavy lifting for calculations. The setup is so
it is a dynamic library that can be used by other projects. It has its own test setup. Only when you
compile from within stellaris you include the tests. Then you can run them with either **ctest** or
run the normal executable.

From the root project you can compile the whole project. Then you can also run tests with ctest.
Those are just the tests for the frontend. (We could maybe implement, that there are run both. Or
you could specify what you want to run). 

The setup for testing within iris is not finished yet. 

# Planet distances
Sun -> Earth = 149.6 million km = 149 600 000
Earth -> Moon = 384,400 km = 384 400
Sun -> Mercury = 57.9 million km = 57 900 000
Sun -> Venus = 108.2 million km = 108 200 000
Sun -> Mars = 227.9 million km = 227 900 000
Sun -> Jupiter = 778.3 million km = 778 300 000
Sun -> Saturn = 1 427 million km = 1 427 000 000
Sun -> Uranus = 2 871 million km = 2 871 000 000
Sun -> Neptune = 4 498 million km = 4 498 000 000
Sun -> Pluto = 5.9 billion km = 5 900 000 000
