# Setup after download

First of all, the [Local Solver](https://www.localsolver.com/) must be installed on your local [Windows](https://www.localsolver.com/docs/last/exampletour/vrp.html) or [Linux](https://www.localsolver.com/docs/last/installation/installationonlinux.html) machine. As well as Python3.10 or newer.

Also, IBM's [ILOG CPLEX Optimization Studio](https://www.ibm.com/products/ilog-cplex-optimization-studio) must be installed on your local machine and must be avaialable through the PATH. Once it's installed, the following should be run once:

Bash (Linux)

```bash
sudo ln -s '/home/<user>/apps/ibm' '/opt/ibm'
sudo ln -s '/home/<user>/apps/localsolver_11_5' '/opt/localsolver_11_5'
echo -e 'PATH="$PATH:/opt/ibm/ILOG/CPLEX_Studio2211/cplex/bin/x86-64_linux"\n' >>~/.bashrc
```

CMD (Admin)

```bash
where cplex | sed 's/\\\\cplex.exe$//' | sed 's/\//\\\\/g'
cd "<last_output>\..\..\..\..\..\.."
python3 "IBM\ILOG\CPLEX_Studio2211\python\setup.py" install --user
```

Execute the following to enter a virtual environment depending on your terminal interpreter:

Bash (Linux)

```bash
python3 -m venv --clear .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python "/opt/ibm/ILOG/CPLEX_Studio2211/python/setup.py" install
python -m pip install localsolver -i "https://pip.localsolver.com"
```

Bash (Windows)

```bash
python3 -m venv --clear .venv
source .venv/Scripts/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install localsolver -i "https://pip.localsolver.com"
```

CMD

```bash
python3 -m venv --clear .venv
.venv\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install localsolver -i "https://pip.localsolver.com"
```

Power Shell

```powershell
python3 -m venv --clear .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install localsolver -i "https://pip.localsolver.com"
```

Unpack the problem instances:

```bash
python -c "import shutil; shutil.unpack_archive('instances.zip')"
```

Execute the main code:

```bash
python -m hybrid_dmrp instances/instancia1Tijuca_0.6.csv
```

### Testing

To run test suites use:

```bash
python -m tests [[test_suite_name] ...]
```

If no other parameter is passed except the script name, all test suites are run. The parameters specify which test suites should run.

### Dependencies

To update the dependencies just insert the new dependencies into ```requirements.in``` and run (in the virtual environment):

```bash
python -m piptools compile --quiet --resolver=backtracking --output-file=requirements.txt requirements.in
```
