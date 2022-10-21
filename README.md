# Setup after download

Execute the following to enter a virtual environment depending on your terminal interpreter:

Bash (Linux)

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Bash (Windows)

```bash
python3 -m venv .venv
source .venv/Scripts/activate
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

CMD

```bash
python3 -m venv .venv
.venv\Scripts\activate.bat
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
```

Power Shell

```powershell
python3 -m venv .venv
.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
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
python -m piptools compile --quiet --output-file=requirements.txt requirements.in
```
