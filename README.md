# nlib2::Modflow Library 

## Overview

Modflow is a C++ framework designed for building modular, graph-based software systems. It enables the creation of flexible and maintainable architectures by organizing code into distinct modules that communicate through well-defined channels. This approach facilitates loose coupling between components, making systems easier to develop, test, and maintain.

## Key Concepts

### Modules

Modules are the core units of computation in Modflow. Each module encapsulates a specific functionality or task and can interact with other modules via channels. Modules can be sources, sinks, or intermediary processors.

- **Source Modules**: Entry points for data into the Modflow system. They do not receive data from other modules but can emit data into the system.
- **Sink Modules**: Exit points for data from the Modflow system. They receive data from other modules but do not emit data to other modules. Instead, they often handle final data processing or output.
- **Intermediary Modules**: Perform data processing, transformation, or routing between source and sink modules.

### Channels

Channels are conduits through which modules communicate. A channel is defined by its data type(s) and serves as a link between emitter (source or intermediary) modules and receiver (intermediary or sink) modules. Channels ensure type-safe data transmission across the system.

- **Emitting on Channels**: Modules can emit data on channels they own or have been granted access to.
- **Receiving from Channels**: Modules can listen to channels and process data emitted by other modules. This is set up through connections that define how data flows from one module to another.

### Connections

Connections define the data flow between modules via channels. They are created based on the types of channels and the compatibility of data types between emitter and receiver modules.

- **Direct Connections**: Created when a module explicitly requests to receive data from a specific channel.
- **Sink Connections**: Special types of connections that link intermediary modules to sink modules for data egress out of the system.

### Resource Management

The `ResourceManager` class in Modflow allows for shared access to resources across modules. Resources can be anything from data buffers to utility objects required by multiple modules. Resource management facilitates efficient sharing and reduces redundancy.

## System Setup and Execution

1. **Define Modules**: Implement the logic for source, sink, and intermediary modules by inheriting from the `Module` class and overriding necessary methods.
2. **Create Modflow Instance**: Instantiate a Modflow object and use it to load modules and define the system architecture.
3. **Declare Channels**: Use source and sink modules to declare entry and exit points for data.
4. **Establish Connections**: Define how modules are connected through channels to dictate the flow of data across the system.
5. **Initiate Data Flow**: Start the system by emitting data into source channels, which then flows through the connected modules according to the defined connections.

## Example: Processing and Merging Data Streams

This example demonstrates how to set up a Modflow system with three modules that process and merge data from two different sources before emitting the merged result to a unique sink. It illustrates the creation and interaction of source, intermediary, and sink modules, the declaration of channels, and the establishment of connections for data flow.

### System Overview

- **FirstProcessor Module**: Processes integer data from the first source, converting it to a string format.
- **SecondProcessor Module**: Processes double data from the second source, also converting it to a string format.
- **Merger Module**: Receives processed data from both FirstProcessor and SecondProcessor, merges this data, and emits the merged result to a sink.

### Modules Implementation

#### FirstProcessor

The `FirstProcessor` module listens for integer data on the `inputData1` channel. Upon receiving data, it processes this data by converting it to a string with a prefix and then emits the processed data on the `toMergerFirst` channel, targeting the `Merger` module.


```cpp
class FirstProcessor : public Module {
public:
    FirstProcessor(const std::shared_ptr<Modflow> &modflow)
        : Module(modflow, "FirstProcessor") {}

    void setupNetwork() override {
        requestConnection<int>("inputData1", &FirstProcessor::processData);
    }

    void processData(int data) {
        std::string processed = "Processed int: " + std::to_string(data);
        emit("toMergerFirst", processed);
    }
};
```

#### SecondProcessor


Similar to the `FirstProcessor`, but it processes double data received from a different source channel.
It converts the data into a string format, adding a prefix for identification. The processed data is then emitted on the `toMergerSecond` channel for the `Merger` module.

```cpp
class SecondProcessor : public Module {
public:
    SecondProcessor(const std::shared_ptr<Modflow> &modflow)
        : Module(modflow, "SecondProcessor") {}

    void setupNetwork() override {
        requestConnection<double>("inputData2", &SecondProcessor::processData);
    }

    void processData(double data) {
        std::string processed = "Processed double: " + std::to_string(data);
        emit("toMergerSecond", processed);
    }
};
```

#### Merger

This module merges string data coming from the `FirstProcessor` and `SecondProcessor` modules.
It synchronize data flows from two sources. It waits to receive string data on two separate channels (`toMergerFirst` and `toMergerSecond`). Upon receiving data from both channels, it merges the strings into a single message and emits this merged message on the `mergedOutput` channel, directed towards a sink. This mechanism demonstrates the module's ability to coordinate data flows and execute conditional logic based on the reception of multiple data inputs.

```cpp
class Merger : public Module {
public:
    Merger(const std::shared_ptr<Modflow> &modflow)
        : Module(modflow, "Merger") {}

    void setupNetwork() override {
        // Establish connections to listen for data from two different channels.
        requestConnection<std::string>("toMergerFirst", &Merger::processFirstData);
        requestConnection<std::string>("toMergerSecond", &Merger::processSecondData);

        // Ensure a sink with the correct types has been declared
        requireSink<std::string> ("mergedOutput");
    }

private:
    std::optional<std::string> firstData;
    std::optional<std::string> secondData;

    void processFirstData(const std::string &data) {
        firstData = data;
        tryEmitMergedData();
    }

    void processSecondData(const std::string &data) {
        secondData = data;
        tryEmitMergedData();
    }

    void tryEmitMergedData() {
        // Checks if both data pieces have been received. If so, it merges and emits them.
        // This conditional logic showcases how modules can perform actions based on the state of their inputs.
        if (firstData && secondData) {
            std::string mergedData = "Merged: " + *firstData + " & " + *secondData;
            // Emitting data to a channel demonstrates how modules can output data to the system.
            emit("mergedOutput", mergedData);

            // Reset optional values for the next merge operation.
            firstData.reset();
            secondData.reset();
        }
    }
};
```

### Modflow Setup

1. **Define the Modflow Derived Class**: Implements the `loadModules` method to load the `FirstProcessor`, `SecondProcessor`, and `Merger` modules.

    ```cpp
    class MyAppFlow : public Modflow {
    protected:
        void loadModules() override {
            loadModule<FirstProcessor>();
            loadModule<SecondProcessor>();
            loadModule<Merger>();
        }
    };
    ```

2. **Main Function Setup**: Demonstrates how to set up sources, sinks, and initiate the data flow.

    - **Declare Source Channels**: Two source channels are declared for the different data types (`int` and `double`).
    - **Declare Sink Channel**: A single sink channel is declared for the merged output.
    - **Emit Data**: Data is emitted on the source channels to start the processing flow.

    ```cpp
    int main() {
        auto modflow = MyAppFlow::create<MyAppFlow>();
        modflow->init();
        
        // Declare source channels for input data
        modflow->sources()->declareSourceChannel<int>("inputData1");
        modflow->sources()->declareSourceChannel<double>("inputData2");

        // Declare a sink channel for the merged output and connect it to an external callback
        modflow->sinks()->declareSink<std::string>("mergedOutput", [](std::string data) {
            std::cout << "Sink received: " << data << std::endl;
        });

        modflow->finalize();

        // Emit data to start the process
        modflow->sources()->callSource("inputData1", 100);
        modflow->sources()->callSource("inputData2", 200.5);

        return 0;
    }
    ```

### Concept Implementation

- **Modules and Channels**: This example demonstrates how modules can define, listen to, and emit data on channels. The `FirstProcessor` and `SecondProcessor` modules process incoming data and emit it to the `Merger` module. The `Merger` module listens on two separate channels, merges the data, and then emits the result to a sink.
- **Source and Sink**: Source channels are used to inject data into the system, while the sink channel serves as the endpoint for the processed data, showcasing the flexibility and power of Modflow's design in handling complex data flows.

This comprehensive example encapsulates the essence of Modflow, illustrating how to build a modular and scalable system with clear data flow between components.
## Conclusion

Modflow provides a robust framework for building modular systems in C++, offering a structured approach to managing complex software architectures. By organizing code into modules and defining clear paths for data flow, developers can create scalable and maintainable systems.
