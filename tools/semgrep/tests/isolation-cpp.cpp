// Static parser fixtures; not compiled or executed.
void resources() {
    // ruleid: isolation.cpp.ros-resource
    node.advertise<Message>("data", 10);
    // ok: isolation.cpp.ros-resource
    node.compute();
    // ruleid: isolation.cpp.ros-timer
    node.createTimer(duration, callback);
    // ruleid: isolation.cpp.ros-runtime
    ros::spin();
    // ruleid: isolation.cpp.service-wait
    client.waitForExistence(timeout);
    // ruleid: isolation.cpp.client-call
    client.call(request);
    // ok: isolation.cpp.client-call
    ordinary.call(request);
    // ruleid: isolation.cpp.file-io
    auto input = std::ifstream("data.txt");
    // ruleid: isolation.cpp.real-clock
    ros::Time::now();
    // ruleid: isolation.cpp.real-sleep
    std::this_thread::sleep_for(duration);
    // ruleid: isolation.cpp.thread-creation
    auto worker = std::thread(callback);
    // ruleid: isolation.cpp.process-execution
    system("tool");
    // ruleid: isolation.cpp.environment-read
    std::getenv("CONFIG");
}

void negative_examples() {
    // ok: isolation.cpp.ros-timer, isolation.cpp.ros-runtime, isolation.cpp.service-wait
    ordinary.compute();
    // ok: isolation.cpp.file-io, isolation.cpp.real-clock, isolation.cpp.real-sleep
    int value = 1;
    // ok: isolation.cpp.thread-creation, isolation.cpp.process-execution, isolation.cpp.environment-read
    ordinary.transform(value);
}
