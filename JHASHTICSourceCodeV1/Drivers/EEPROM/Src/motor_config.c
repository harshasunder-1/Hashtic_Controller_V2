#include "motor_config.h"

void MotorConfig_Init(void)
{
    HAL_FLASH_Unlock();

    EE_Init(EE_CONDITIONAL_ERASE);

    HAL_FLASH_Lock();
}

EE_Status MotorConfig_Write(uint32_t motorID,int32_t sign,int32_t position){
    EE_Status status = EE_OK;

    HAL_FLASH_Unlock();

    status = EE_WriteVariable32bits(
        EE_MOTOR_ID_ADDR,
        motorID
    );

    if (status != EE_OK){
      return status;
    }

    status = EE_WriteVariable32bits(
        EE_MOTOR_ID_COPY_ADDR,
        motorID
    );
    
    if (status != EE_OK){
      return status;
    }

    status = EE_WriteVariable32bits(
        EE_SIGN_ADDR,
        (uint32_t)sign
    );
    
    if (status != EE_OK){
      return status;
    }

    status = EE_WriteVariable32bits(
        EE_SIGN_COPY_ADDR,
        (uint32_t)sign
    );
    
    if (status != EE_OK){
      return status;
    }

    status = EE_WriteVariable32bits(
        EE_POSITION_ADDR,
        (uint32_t)position
    );
    
    if (status != EE_OK){
      return status;
    }

    status = EE_WriteVariable32bits(
        EE_POSITION_COPY_ADDR,
        (uint32_t)position
    );
    
    if (status != EE_OK){
      return status;
    }

    HAL_FLASH_Lock();

    return EE_OK;
}

EE_Status MotorConfig_Read(EEConfig *ee){
  
    EE_Status status = EE_OK;
    
    status = EE_ReadVariable32bits(EE_MOTOR_ID_ADDR,(uint32_t *)&ee->motorID); 
    if (status != EE_OK){return status;}

    status = EE_ReadVariable32bits(EE_MOTOR_ID_COPY_ADDR,(uint32_t *)&ee->motorIDCopy);
    if (status != EE_OK){return status;}

    status = EE_ReadVariable32bits(EE_SIGN_ADDR,(uint32_t *)&ee->signforCW);
    if (status != EE_OK){return status;}

    status = EE_ReadVariable32bits(EE_SIGN_COPY_ADDR,(uint32_t *)&ee->signforCWCopy);
    if (status != EE_OK){return status;}

    status = EE_ReadVariable32bits(EE_POSITION_ADDR,(uint32_t *)&ee->positionInPod);
    if (status != EE_OK){return status;}

    status = EE_ReadVariable32bits(EE_POSITION_COPY_ADDR,(uint32_t *)&ee->positionInPodCopy);
    if (status != EE_OK){return status;}
    
    return EE_OK;
}


uint8_t CheckEEConfigValues(EEConfig *ee){

     ee->eeReadStatus = EEPROM_VALUES_OK;
    //motor  
    if (ee->motorID  < 1 || ee->motorID  > 20){
      ee->eeReadStatus = EEPROM_IDS_OUT_OF_RANGE;
      return  ee->eeReadStatus;
    }
    
    if (ee->motorIDCopy  < 1 || ee->motorIDCopy  > 20){
        ee->eeReadStatus = EEPROM_IDS_OUT_OF_RANGE;
        return  ee->eeReadStatus;
    }
    
    if (ee->motorID != ee->motorIDCopy){
        ee->eeReadStatus = EEPROM_IDS_NOT_MATCHING;
        return  ee->eeReadStatus;
    }
    
    //sign for CW
    if (ee->signforCW != 1 && ee->signforCW != -1){
        ee->eeReadStatus = EEPROM_CWSIGN_OUT_OF_RANGE;
        return  ee->eeReadStatus;
    }
    if (ee->signforCWCopy != 1 && ee->signforCWCopy != -1){
        ee->eeReadStatus = EEPROM_CWSIGN_OUT_OF_RANGE;
        return  ee->eeReadStatus;
    }
   
    if (ee->signforCW != ee->signforCWCopy){
        ee->eeReadStatus = EEPROM_CWSIGNS_NOT_MATCHING;
        return  ee->eeReadStatus;
    }
    
    // position in pod
    if (ee->positionInPod != ee->positionInPodCopy){
        ee->eeReadStatus = EEPROM_SIDES_NOT_MATCHING;
        return  ee->eeReadStatus;
    }

    if (ee->positionInPod != LEFT_SIDE && ee->positionInPod != RIGHT_SIDE){
        ee->eeReadStatus = EEPROM_SIDES_OUT_OF_RANGE;
        return  ee->eeReadStatus;
    }
    
    if (ee->positionInPodCopy != LEFT_SIDE && ee->positionInPodCopy != RIGHT_SIDE){
        ee->eeReadStatus = EEPROM_SIDES_OUT_OF_RANGE;
        return  ee->eeReadStatus;
    }
    
    return  ee->eeReadStatus;
}