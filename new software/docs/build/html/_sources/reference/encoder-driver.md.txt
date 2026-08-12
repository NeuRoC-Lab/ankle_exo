# Reading encoder positions

The framework uses an `EncoderDriver` to manage the two joint encoders.

The driver handles initialization, validation of encoder readings,
rollover correction, and conversion from counts to degrees.

## Encoder driver

```{doxygenclass} EncoderDriver
:project: framework
```

The important part for normal application code is that the driver
must first be initialized.

```{doxygenfunction} EncoderDriver::begin
:project: framework
```

After initialization, call `sample()` whenever a new encoder
measurement is required.

```{doxygenfunction} EncoderDriver::sample
:project: framework
```