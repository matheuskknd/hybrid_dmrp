# Tunning scenario for IRace
---

This directory contains the files used by IRace to tune the optimization algorithm.

## Testing the scenario and environment

Inside this file directory execute one of the following:

Linux

```bash
chmod +x target-runner.sh
irace --target-runner target-runner.sh --scenario scenario.txt --check
```

Windows

```cmd
irace --target-runner target-runner.cmd --scenario scenario.txt --check
```

## Executing the scenario

Always test the enviroment before running the scenario, some parts of the target-runner script first run won't work if parallelized. So, inside this file directory, execute one of the following:

Linux

```bash
irace --target-runner target-runner.sh --scenario scenario.txt --parallel <number_of_parallel_executions>
```

Windows

```cmd
irace --target-runner target-runner.cmd --scenario scenario.txt --parallel <number_of_parallel_executions>
```

## Installing IRace on Ubuntu 20.04.5 LTS (Focal)

In your ```/etc/apt/sources.list``` file add the following line:

```
deb https://cloud.r-project.org/bin/linux/ubuntu focal-cran40/
```

Then execute:

```bash
sudo apt update
sudo apt install r-base
source ~/.bashrc
R --version
```

Then, enter the R interpreter and execute:

```R
install.packages("irace")
```

If warning messages appear (like shown below), typing "yes" for both should work:

```
Warning in install.packages("irace") :
  'lib = "/usr/local/lib/R/site-library"' is not writable
Would you like to use a personal library instead? (yes/No/cancel) yes
Would you like to create a personal library
‘~/R/x86_64-pc-linux-gnu-library/3.6'
to install packages into? (yes/No/cancel) yes
```

This R line should rise no error if the package is correctly installed and show where the package is installed:

```R
library("irace")
system.file(package = "irace")
q()
```

Add the package script path to the environment variable ```$PATH```:

```bash
echo 'export PATH="$PATH:<irace_package_path>/bin"' >>~/.bashrc
source ~/.bashrc
irace --version
```
