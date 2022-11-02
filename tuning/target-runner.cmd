@ECHO OFF

:: Forward the parameters correctly, even if they have inner spaces
:: The R/IRace passes file parameters with inner spaces replaced by "?"
:: Then Bash script converts it back before calling the module
CALL bash target-runner.sh %*
