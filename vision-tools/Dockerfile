FROM skynet/base
RUN apt-get update && apt-get install -y python3-tk python3-pip \
    build-essential \
	ca-certificates \
    git
RUN python3 -m pip install fpdf pyproj scikit-learn pandas openpyxl
