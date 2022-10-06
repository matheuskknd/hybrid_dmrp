# Setup after download

Execute the following to enter a virtual environment depending on your terminal interpreter:

Bash

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

Then the following to execute the main code:

```bash
python -m brkga
```

### Dependencies

To update the dependencies just insert the new dependencies into ```requirements.in``` and run (in the virtual environment):

```bash
python -m piptools compile --quiet --output-file=requirements.txt requirements.in
```
