# Setup after download

First of all, the [Local Solver](https://www.localsolver.com/) must be installed on your local [Windows](https://www.localsolver.com/docs/last/exampletour/vrp.html) or [Linux](https://www.localsolver.com/docs/last/installation/installationonlinux.html) machine. As well as Python3.10 or newer.

Execute the following to enter a virtual environment depending on your terminal interpreter:

Bash (Linux)

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install localsolver -i "https://pip.localsolver.com"
```

Bash (Windows)

```bash
python3 -m venv .venv
source .venv/Scripts/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install localsolver -i "https://pip.localsolver.com"
```

CMD

```bash
python3 -m venv .venv
.venv\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m pip install localsolver -i "https://pip.localsolver.com"
```

Power Shell

```powershell
python3 -m venv .venv
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

### Dependencies

To update the dependencies just insert the new dependencies into ```requirements.in``` and run (in the virtual environment):

```bash
python -m piptools compile --quiet --resolver=backtracking --output-file=requirements.txt requirements.in
```
