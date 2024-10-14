# Traffic light detection

The training is configured as DVC experiment.
More details about dvc experiments can be found [here](../../../../doc/development/dvc.md).

## Training

The training can be run with `dvc exp run`.
The params defined in `params.yaml` can be overridden via commandline with e.g. `dvc exp run --set-param train.epochs=24`.

### Available paramters

| paramater    | description                   |
|--------------|-------------------------------|
| train.epochs | Number of epochs for training |
