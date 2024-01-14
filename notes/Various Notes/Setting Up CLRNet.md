CLRNet is the main neural network implementation we want to use for lane detection. The network works, but needs some tweaking to get it set up.

This repository contains a shell script `setup_clrnet.sh` that configures an environment suitable for CLRNet to run in. When we set up the Docker Compose environment to have detection running in a separate Docker container alongside the other ROS2 nodes, this wlil be a lot cleaner.

- CLRNet needs Python 3.8 to run. Use `venv` or Docker to set up a Python 3.8 environment.
- CLRNet requires the following PyTorch dependencies (and versions) to be installed before the build starts:
	- `torch==1.8.x`
	- `torchvision==0.9.x`
	- where x can be any patch version number

Clone CLRNet from the following Git repository:
[https://github.com/Turoad/CLRNet.git](https://github.com/Turoad/CLRNet.git)

- The `requirements.txt` file in the repository is broken, so the install script won't work by default. 
	- `torch` and `torchvision` are specified in the requirements file, meaning they will likely auto-update to later versions of the packages and will break. Remove these dependencies from the requirements file as they are already installed.
	- `numpy` is used as an implicit dependency, meaning the installer downloads the latest version. CLRNet has a handful of instances of `np.bool`, a data type that was removed in NumPy 1.20 in favour of Python's native `bool` type. Replace `numpy` with `numpy==1.19` to install a version of NumPy with `np.bool` still included.
	- CLRNet uses `scikit-learn` as a dependency, however, it specifies `sklearn`, the old and deprecated name for the package, which raises an exception on installation. Replace `sklearn` with `scikit-learn` to fix this.

The aforementioned shell script does all of this automatically. :)
```bash
sed -i '/torch/d' requirements.txt
sed -i '/torchvision/d' requirements.txt
sed -i '1s/^/numpy==1.19\n/' requirements.txt
sed -i 's|sklearn|scikit-learn|g' requirements.txt
```

Once the dependencies are set up run `python setup.py build develop` to build the CLRNet Python module. Once it's built, you can copy the new `clrnet` directory into your workspace and use a relative import to use it in your code. 🎉