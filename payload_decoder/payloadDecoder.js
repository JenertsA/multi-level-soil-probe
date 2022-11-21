function decodeUplink(input) {
    var rawBytes = input.bytes;
    
    var data = {};
    //data.raw_payload = rawBytes;
  
    //array for all the reading
    var sensors = [];
    var metaData = [];
    
    for(var currentByte =0; currentByte < rawBytes.length; currentByte++){
      
      //start by reading sens type and level
      var sensTypeId = rawBytes[currentByte] & (0x0F);
      var sensLevel = rawBytes[currentByte] >> 4;
      
      switch (sensTypeId){
        case 0x00:
          //soil temperature sensor
          
          //--THIS IS NOW DONE IN FUNCTION decodeTemp(upper, lower)--//
          // //info is stored in next two bytes
          // var tempUpper = rawBytes[currentByte + 1];
          // var tempLower = rawBytes[currentByte + 2];
          // //skip these two bytes in for
          // currentByte = currentByte + 2; 
          
          // //re assemble and take offset into account
          // var tempC =  roundToTwo((((tempUpper << 8) | tempLower) / 100) - 40);
          //----///----//
          
          var soilTempC = decodeTemp(rawBytes[currentByte + 1], rawBytes[currentByte + 2]);
          currentByte = currentByte + 2; 
          
          var soilTempSensData = {
            lvl:sensLevel,
            typeId:sensTypeId,
            type:"soil_temperature",
            value:soilTempC
          };
          sensors.push(soilTempSensData);
          break;
          
        case 0x01: 
          //air temperature 
          var airTempC = decodeTemp(rawBytes[currentByte + 1], rawBytes[currentByte + 2]);
          currentByte = currentByte + 2; 
          
          var airTempSensData = {
            lvl:sensLevel,
            typeId:sensTypeId,
            type:"air_temperature",
            value:airTempC
          };
          sensors.push(airTempSensData);
        break;
        
        case 0x0A:
          //battery voltage
          var batteryVoltage = rawBytes[currentByte + 1] / 100 + 3.3;
          currentByte = currentByte +1;
          var batteryVoltageData = {
            typeId:sensTypeId,
            type:"battery_voltage",
            value: batteryVoltage,
          };
          metaData.push(batteryVoltageData);
        break;
        
        case 0x0F:
          data.error = {
            error: "Node not functioning",
          };
        break;
      }
    }
    
    data.sensors = sensors;
    data.metaData = metaData;
    return {
      data: data,
      warnings: []
    };
  }
  
  function decodeTemp(tempUpper, tempLower){
    //re assemble and take offset into account
    return  roundToTwo((((tempUpper << 8) | tempLower) / 100) - 40);
  }
  
  function roundToTwo(num) {    
      return +(Math.round(num + "e+2")  + "e-2");
  }