def sideWall(channel):
    v = channel.voltage
    value = channel.value
    print(f'ADC voltage: {v:.3f} V')
    print(f'ADC value: {value}')
    
    #TODO figure out threshold
    threshold = 0
    if(value > threshold):
        return True
    else:
        return False
def frontWall(channel1, channel2):
    v1 = channel1.voltage
    value1 = channel1.value
    v2 = channel2.voltage
    value2 = channel2.value
    print(f'Receiver 1 ADC voltage: {v1:.3f} V')
    print(f'Receiver 1 ADC value: {value1}')
    print(f'Receiver 2 ADC voltage: {v2:.3f} V')
    print(f'Receiver 2 ADC value: {value2}')
    
    #TODO figure out threshold
    threshold = 0
    #options:
    #-average
    #-minimum value
    if((value1+value2)/2 > threshold):
        return True
    else:
        return False
    