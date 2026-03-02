#include "BH1750.h"
#include "../Libs/SSD1306/ssd1306.h"

extern I2C_HandleTypeDef hi2c1;

int8_t BH1750IsNull(BH1750 *sensor){
	if (sensor == NULL){
		return BH1750_IS_NULL;
	}	
	
	if(sensor->buff == NULL || sensor->ret == NULL || sensor->port == NULL){
		sensor->lastError = BH1750_NOT_INITIALIZED;
		return BH1750_NOT_INITIALIZED;
	}
	
	return BH1750_OK;
}

BH1750* BH1750Init(const I2C_HandleTypeDef *port, const uint8_t address){	
	//Выделяем память для структуры, описывающей сенсор
	BH1750 *sensor = (BH1750*)malloc(sizeof(BH1750));
	
	// Верни NULL при ошибке
	if(sensor == NULL){
		return NULL;
	}
	
	//Выделяем память для буфера данных
	sensor->buff = (uint8_t*)calloc(3,1);
	
	// Верни NULL при ошибке, очисти выделенные ресурсы
	if(sensor->buff == NULL){
		free(sensor);
		return NULL;
	}
	
	//Выделяем память для перечисления со статусом полученного ответа на последний запрос
	sensor->ret = (HAL_StatusTypeDef*)malloc(sizeof(HAL_StatusTypeDef));
	
	// Верни NULL при ошибке, очисти выделенные ресурсы
	if(sensor->ret == NULL){	
		free(sensor->buff);
		free(sensor);
		return NULL;
	}
	
	//Настройка структуры
	sensor->port = (I2C_HandleTypeDef *)port;
	sensor->address = address;
	
	// Включить сенсор
	sensorPowerSwitch(sensor, BH1750_POWER_UP);
	
	// Верни NULL при ошибке
	if(*sensor->ret != HAL_OK){
		return NULL;
	}
	 
	// Сбросить контроллер
	resetSensor(sensor);
	
	// Верни NULL при ошибке
	if(*sensor->ret != HAL_OK){
		return NULL;
	} 
	
	return sensor;
}

BH1750_STATUS readHighResolutionDataMode1(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_HIGH_RES_MODE1;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений
	HAL_Delay(200);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 3, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	sensor->measure = sensor->buff[0] << 8 | sensor->buff[1];
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS readHighResolutionDataMode2(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_HIGH_RES_MODE2;

	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений
	HAL_Delay(200);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 3, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	sensor->measure = (sensor->buff[0] << 8 | sensor->buff[1]) / 2.0;
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

void readHighResolutionData(BH1750 *sensor, const MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		readHighResolutionDataMode1(sensor); //Высокая точность, 1 люкс
	}else{
		readHighResolutionDataMode2(sensor); //Высокая точность, 0.5 люкс
	}
}

BH1750_STATUS readLowResolutionData(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_LOW_RES_MODE;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений
	HAL_Delay(30);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 3, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	sensor->measure = sensor->buff[0];
		
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS readOnceHighResolutionDataMode1(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_ONCE_HIGH_RES_MODE1;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений
	HAL_Delay(200);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 3, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	sensor->measure = sensor->buff[0] << 8 | sensor->buff[1];
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS readOnceHighResolutionDataMode2(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_HIGH_RES_MODE2;

	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений
	HAL_Delay(200);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 3, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	sensor->measure = (sensor->buff[0] << 8 | sensor->buff[1]) / 2.0;
		
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

void readOnceHighResolutionData(BH1750 *sensor, const MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		readOnceHighResolutionDataMode1(sensor); //Высокая точность, 1 люкс
	}else{
		readOnceHighResolutionDataMode2(sensor); //Высокая точность, 0.5 люкс
	}
}

BH1750_STATUS readOnceLowResolutionData(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_ONCE_LOW_RES_MODE;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений
	HAL_Delay(30);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 3, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	sensor->measure = sensor->buff[0];
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS sensorPowerSwitch(BH1750 *sensor, const uint8_t mode){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	if(mode != BH1750_POWER_UP && mode != BH1750_POWER_DOWN){
		sensor->lastError = BH1750_UNKNOWN_COMMAND;
		return BH1750_UNKNOWN_COMMAND;
	}
	
	//Указываем команду
	sensor->buff[0] = mode;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, 75);
	
	//Если получили ошибку - сообщаем
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS resetSensor(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_RESET;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, 30);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
		
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS setSensorSensitivity(BH1750 *sensor, const uint8_t sensitivity){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Щаполняем буфер команд
	sensor->buff[0] = 0x40;
	sensor->buff[1] = sensitivity;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 2, 300);
	
		//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

void BH1750Deleter(BH1750 *sensor){
	free(sensor->buff);
	free(sensor->ret);
	free(sensor);
}
/////////////////////////////NON-BLOCKING MODE WITH INTERRUPTIONS/////////////////////////////
BH1750_STATUS transmitHighResolutionDataMode1IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_HIGH_RES_MODE1;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;	
}

BH1750_STATUS transmitHighResolutionDataMode2IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_HIGH_RES_MODE2;

	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

void transmitHighResolutionDataIT(BH1750 *sensor, const MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		transmitHighResolutionDataMode1IT(sensor); //Высокая точность, 1 люкс
	}else{
		transmitHighResolutionDataMode2IT(sensor); //Высокая точность, 0.5 люкс
	}
}


BH1750_STATUS transmitLowResolutionDataIT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_LOW_RES_MODE;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS transmitOnceHighResolutionDataMode1IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_ONCE_HIGH_RES_MODE1;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS transmitOnceHighResolutionDataMode2IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_HIGH_RES_MODE2;

	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}

	sensor->lastError = BH1750_OK;
	return BH1750_OK;		
}

void transmitOnceHighResolutionDataIT(BH1750 *sensor, const MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		transmitOnceHighResolutionDataMode1IT(sensor); //Высокая точность, 1 люкс
	}else{
		transmitOnceHighResolutionDataMode2IT(sensor); //Высокая точность, 0.5 люкс
	}
}

BH1750_STATUS transmitOnceLowResolutionDataIT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_ONCE_LOW_RES_MODE;

	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}

	sensor->lastError = BH1750_OK;
	return BH1750_OK;	
}


BH1750_STATUS sensorPowerSwitchIT(BH1750 *sensor, const uint8_t mode){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	if(mode != BH1750_POWER_UP && mode != BH1750_POWER_DOWN){
		sensor->lastError = BH1750_UNKNOWN_COMMAND;
		return BH1750_UNKNOWN_COMMAND;
	}
	
	//Указываем команду
	sensor->buff[0] = mode;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - сообщаем
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;	
}

BH1750_STATUS resetSensorIT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_RESET;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
		
	sensor->lastError = BH1750_OK;
	return BH1750_OK;	
}

BH1750_STATUS setSensorSensitivityIT(BH1750 *sensor, const uint8_t sensitivity){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Щаполняем буфер команд
	sensor->buff[0] = 0x40;
	sensor->buff[1] = sensitivity;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, sensor->buff, 2);
	
		//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;	
}

BH1750_STATUS recieveIT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive_IT(sensor->port, sensor->address, sensor->buff, 3);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

/////////////////////////////NON-BLOCKING MODE WITH DMA/////////////////////////////
BH1750_STATUS transmitHighResolutionDataMode1DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_HIGH_RES_MODE1;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;	
}

BH1750_STATUS transmitHighResolutionDataMode2DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_HIGH_RES_MODE2;

	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

void transmitHighResolutionDataDMA(BH1750 *sensor, const MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		transmitHighResolutionDataMode1DMA(sensor); //Высокая точность, 1 люкс
	}else{
		transmitHighResolutionDataMode2DMA(sensor); //Высокая точность, 0.5 люкс
	}
}


BH1750_STATUS transmitLowResolutionDataDMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_LOW_RES_MODE;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS transmitOnceHighResolutionDataMode1DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_ONCE_HIGH_RES_MODE1;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS transmitOnceHighResolutionDataMode2DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_HIGH_RES_MODE2;

	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}

	sensor->lastError = BH1750_OK;
	return BH1750_OK;		
}

void transmitOnceHighResolutionDataDMA(BH1750 *sensor, const MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		transmitOnceHighResolutionDataMode1DMA(sensor); //Высокая точность, 1 люкс
	}else{
		transmitOnceHighResolutionDataMode2DMA(sensor); //Высокая точность, 0.5 люкс
	}
}

BH1750_STATUS transmitOnceLowResolutionDataDMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_ONCE_LOW_RES_MODE;

	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}

	sensor->lastError = BH1750_OK;
	return BH1750_OK;	
}


BH1750_STATUS sensorPowerSwitchDMA(BH1750 *sensor, const uint8_t mode){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	if(mode != BH1750_POWER_UP && mode != BH1750_POWER_DOWN){
		sensor->lastError = BH1750_UNKNOWN_COMMAND;
		return BH1750_UNKNOWN_COMMAND;
	}
	
	//Указываем команду
	sensor->buff[0] = mode;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - сообщаем
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;	
}

BH1750_STATUS resetSensorDMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_RESET;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, sensor->buff, 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
		
	sensor->lastError = BH1750_OK;
	return BH1750_OK;	
}

BH1750_STATUS setSensorSensitivityDMA(BH1750 *sensor, const uint8_t sensitivity){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Щаполняем буфер команд
	sensor->buff[0] = 0x40;
	sensor->buff[1] = sensitivity;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, sensor->buff, 2);
	
		//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;	
}

BH1750_STATUS recieveDMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	BH1750_STATUS status = BH1750IsNull(sensor);
	if(status != BH1750_OK){
		return status;
	}
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive_DMA(sensor->port, sensor->address, sensor->buff, 3);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}