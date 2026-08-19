FROM ubuntu:24.04 AS build

ARG ARDUPILOT_COMMIT=e0652af4c0c9657c04672ed3e21b71de75c74763
ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential ca-certificates ccache g++ git \
    python3 python3-dev python3-pip python3-venv && \
    rm -rf /var/lib/apt/lists/*
RUN git clone --filter=blob:none https://github.com/ArduPilot/ardupilot.git /ardupilot && \
    cd /ardupilot && git checkout "${ARDUPILOT_COMMIT}" && \
    git submodule update --init --recursive --depth 1
RUN python3 -m venv /opt/ardupilot-venv && \
    /opt/ardupilot-venv/bin/pip install --no-cache-dir \
      empy==3.3.4 future lxml pexpect pymavlink
ENV PATH="/opt/ardupilot-venv/bin:${PATH}"
RUN cd /ardupilot && ./waf configure --board sitl && ./waf plane

FROM ubuntu:24.04
RUN DEBIAN_FRONTEND=noninteractive apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    libexpat1 libpython3.12 libstdc++6 python3-minimal && \
    rm -rf /var/lib/apt/lists/*
COPY --from=build /ardupilot/build/sitl/bin/arduplane /opt/ardupilot/arduplane
COPY --from=build /opt/ardupilot-venv /opt/ardupilot-venv
COPY ardupilot/uvd.parm /opt/ardupilot/uvd.parm
COPY ardupilot/entrypoint.sh /opt/ardupilot/entrypoint.sh
COPY ardupilot/session.py /opt/ardupilot/session.py
ENTRYPOINT ["/opt/ardupilot/entrypoint.sh"]
