@ECHO OFF

:: Forward the parameters correctly, even if they have inner spaces
:: The R/IRace passes file parameters with inner spaces replaced by "?"
:: Bash converts "?" inside parameters to inner spaces by default for files
CALL bash target-runner.sh %*
