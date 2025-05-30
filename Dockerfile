FROM ghcr.io/ad-sdl/madsci:latest

LABEL org.opencontainers.image.source=https://github.com/AD-SDL/ur_module
LABEL org.opencontainers.image.description="Drivers and REST API's for the UR robots"
LABEL org.opencontainers.image.licenses=MIT

#########################################
# Module specific logic goes below here #
#########################################

RUN mkdir -p ur_module

COPY ./src ur_module/src
COPY ./README.md ur_module/README.md
COPY ./pyproject.toml ur_module/pyproject.toml

RUN --mount=type=cache,target=/root/.cache \
    pip install -e ./ur_module

CMD ["python", "ur_module/src/ur_rest_node.py"]

#########################################
