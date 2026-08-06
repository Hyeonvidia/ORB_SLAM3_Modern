FROM python:3.12-slim
RUN pip install --no-cache-dir evo==1.31.1 && evo_ape --help >/dev/null
WORKDIR /work
