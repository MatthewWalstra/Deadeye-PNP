# Deadeye Template

Deadeye custom pipeline daemon starter project

## Getting Started

This project comes with an example custom pipeline in `custom_pipeline.cpp` to demonstrate usage.

You can either install build dependencies on your local machine (see the [Dockerfile](https://github.com/strykeforce/deadeye/blob/main/docker/Dockerfile) for the `j3ff/deadeye-cpp-base` image) or use Docker.

1. Edit the `Dockerfile` to update CMake with your desired changes to unit id or pipeline class.
2. The NetworkTables address compile into the executable by CMake will be overridden by the `DEADEYE_NT_SERVER` environment variable set in `docker-compose.yml`.
3. Build the `custom-deadeye` image with: `docker compose build`.
4. Run the pipeline, admin server, and NetworkTables services with: `docker compose up`. This will print any logging info you need to standard output. CTRL-C will stop all services.
5. View the admin server at http://127.0.0.1:8080/
6. The NetworkTables service will persist your pipeline settings to the `nt/` directory.
