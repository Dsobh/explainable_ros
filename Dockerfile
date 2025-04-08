FROM mgons/llama_ros:humble

#Create workspace and copy repository files
WORKDIR /root/ros2_ws
SHELL ["/bin/bash", "-c"]
COPY . /root/ros2_ws/src

#Install dependencies
RUN pip3 install -r /src/requirements.txt

#Colcon and source repository
RUN colcon build
RUN echo "source /install/setup.bash" >> ~/.bashrc


