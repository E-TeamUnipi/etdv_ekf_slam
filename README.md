# Pacsim Workspace

This repo works as a sort of wrapper for the pacsim simulator.

## How to run
### Setup
- be sure to have `Docker` installed
- Clone this repo: `git clone git@github.com:E-TeamUnipi/pacsim_ws.git`
- Enter the directory: `cd pacsim_ws`
- Initialize submodules (pacsim): `git submodule init; git submodule update`
### Build
- Run `docker compose build` (can take more than 10 minutes) 
### Execute
- Run `docker compose up`

### or build and execute
- Run `docker compose up --build`


## Foxglove visualization
- Download/run foxglove
- Connect to `localhost:8765`
