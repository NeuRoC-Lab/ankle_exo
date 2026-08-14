## Topic guidelines

A topic acts as a node that holds a snapshot of a variable, it can be updated (`.publish()`) and its latest value can be probed with (`.latest()`).

### Motivations

Instead of having each controller sample from the peripherals (encoder, load cells) directly, we chose to design a uniform access method that periodically updates its internal state so that controllers can access the latest value held by the Topic. 

A topic implements : 
* `.latest()` : returns the latest value (snapshot) in the state store
* `.publish(T value)` : updates the state store with the given `value`
> Note : for motor topics the method is slightly different and has signature `.apply(MotorCmd& cmd)`


For more informations look at {ref}`the Topic reference page <topic-class>`