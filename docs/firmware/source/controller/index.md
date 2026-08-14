# Creating your custom motor controller and registering it. 

## Programming guidelines

Motor-related controllers should be added onto {ref}`Driver.h <motorcontrollers-h>` which contains all the existing controller classes such as 

```{doxygenclass} MotorCommandTask
:project: framework
```

```{doxygenclass} MotorCanReceiver
:project: framework
```

```{doxygenclass} MotorMetaCommandTask
:project: framework
```

```{doxygenclass} JointLimitController
:project: framework
```

```{doxygenclass} TransparentModeController
:project: framework
```

The Controller class you are designing _MUST_ inherit from the `ITask` class

The declaration should look as follows : 

Note : 
* `[TopicTypeX]` is a placeholder for the type of the topic, for example `EncoderPositions` for the encoder topic. 
* `[topicXMember]` is a placeholder for the name of the member holding a reference to the Topic `Topic<[TopicTypeX]>`
```c++
class YourController final : public ITask { 

public: 

    YourController(Topic<[TopicType1]>& [topic1Member], 
                   Topic<[TopicType2]>& [topic2Member],
                 /* etc... */ 
                  )  
                 : 
                m_topic1Member(topic1Member),
                m_topic2Member(topic2Member),
                /* etc... */       
                { }                  
```
...
```c++
void update(uint32_t nowUs) override
    {
    // your update logic goes here
    // the nowUs parameter can be used to track the current time ( micros() )
    }
```
...
```c++
private:
Topic<[TopicType1]>& [topic1Member];
Topic<[TopicType2]>& [topic2Member];
/*etc*/
// and define any other private fields as well
};
```

For more information about the Topic class, have a look at its definition in {ref}`MessageBus.h <topic-class>`

### Registering the controller to activate it

Once your controller is defined, you need to register it properly in `main.cpp`

1. Instanciate a controller object in the global scope (outside `setup()` or `loop()`). For e.g to create an instance of `JointLimitController`
```c++
JointLimitController jointController(
    encoderTopic,
    loadCellTopic,
    leftMotorTopic,
    leftMotorCommandTopic,
    leftMotorMetaCommandTopic
);
```

2. Add the controller to the `Scheduler` as follows :
Using our `JointLimitController` example again, you can add the `jointController` to the scheduler by adding : 
```c++
scheduler.add(jointController);
```

This will ensure the `jointController::update()` method gets called at every loop iteration. To rate-limit the execution of the `update()` function you need to implement a non-blocking delay inside the update() function itself. For example : 

```c++
    void update(uint32_t nowUs) override
    {
        static constexpr uint32_t PERIOD_US = 10'000;

        if (nowUs - m_previousUs < PERIOD_US)
            return;

        m_previousUs += PERIOD_US;
    ... // rest of update logic goes here
    }
```



