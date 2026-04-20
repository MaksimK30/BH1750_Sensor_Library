#include "BH1750.h"

extern I2C_HandleTypeDef hi2c1;

bool BH1750_IsNull(BH1750 *sensor){
	if (sensor == NULL){
		return true;
	}	
	
	if(sensor->buff == NULL || sensor->ret == NULL || sensor->port == NULL){
		sensor->lastError = BH1750_NOT_INITIALIZED;
		return true;
	}
	
	return false;
}

BH1750* BH1750_Init(I2C_HandleTypeDef *port, uint8_t address){	
	//Выделяем память для структуры, описывающей сенсор
	BH1750 *sensor = (BH1750*)malloc(sizeof(BH1750));
	
	// Верни NULL при ошибке
	if(sensor == NULL){
		return NULL;
	}
	
	//Настройка структуры
	sensor->port = port;
	sensor->address = address;
	sensor->sensitivity = BH1750_DEFAULT_SENSITIVITY;
	
	//Выделяем память для буфера данных
	sensor->buff = (uint8_t*)calloc(2, sizeof(uint8_t));
	
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
	
	// Включить сенсор
	BH1750_SensorPowerSwitch(sensor, BH1750_POWER_UP);
	
	// Верни NULL при ошибке
	if(*sensor->ret != HAL_OK){
		free(sensor->ret);
		free(sensor->buff);
		free(sensor);
		return NULL;
	}
	 
	// Сбросить контроллер
	BH1750_ResetSensor(sensor);
	
	// Верни NULL при ошибке
	if(*sensor->ret != HAL_OK){
		free(sensor->ret);
		free(sensor->buff);
		free(sensor);
		return NULL;
	} 
	
	//Сбросим чувствительность на значение по умолчанию.
	BH1750_SetSensorSensitivity(sensor, BH1750_DEFAULT_SENSITIVITY);
	
	// Верни NULL при ошибке
	if(*sensor->ret != HAL_OK){
		free(sensor->ret);
		free(sensor->buff);
		free(sensor);
		return NULL;
	} 
	
	return sensor;
}

void BH1750_GetLux(BH1750 *sensor, BH1750_MEASUREMENT_RESOLUTION_MODE mode){
	float correction = (mode == BH1750_ONE_LX ? 
												((float)BH1750_DEFAULT_SENSITIVITY / sensor->sensitivity) : 
												((float)BH1750_DEFAULT_SENSITIVITY / sensor->sensitivity) / 2.0f
										);
	sensor->measure = ((uint16_t)(sensor->buff[0] << 8 | sensor->buff[1])) / 1.2f * correction;
}
BH1750_STATUS BH1750_ReadHighResolutionData_Mode1(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_HIGH_RES_MODE1;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений, время вычисления определим динамически
	HAL_Delay(3.2	 *  sensor->sensitivity);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 2, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	BH1750_GetLux(sensor, BH1750_ONE_LX);
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS BH1750_ReadHighResolutionData_Mode2(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_HIGH_RES_MODE2;

	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений, время вычисления определим динамически
	HAL_Delay(3.2	 *  sensor->sensitivity);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 2, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	BH1750_GetLux(sensor, BH1750_HALF_LX);

	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

void BH1750_ReadHighResolutionData(BH1750 *sensor, BH1750_MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		BH1750_ReadHighResolutionData_Mode1(sensor); //Высокая точность, 1 люкс
	}else{
		BH1750_ReadHighResolutionData_Mode2(sensor); //Высокая точность, 0.5 люкс
	}
}

BH1750_STATUS BH1750_ReadLowResolutionData(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		sensor->lastError = BH1750_NOT_INITIALIZED;
		return BH1750_NOT_INITIALIZED;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_LOW_RES_MODE;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений, время вычисления определим динамически
	HAL_Delay(0.45 * sensor->sensitivity);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 2, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	BH1750_GetLux(sensor, BH1750_ONE_LX);
		
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS BH1750_ReadOnceHighResolutionData_Mode1(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_ONCE_HIGH_RES_MODE1;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений, время вычисления определим динамически
	HAL_Delay(3.2	 *  sensor->sensitivity);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 2, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	BH1750_GetLux(sensor, BH1750_ONE_LX);
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS BH1750_ReadOnceHighResolutionData_Mode2(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_ONCE_HIGH_RES_MODE2;

	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений, время вычисления определим динамически
	HAL_Delay(3.2	 *  sensor->sensitivity);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 2, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	BH1750_GetLux(sensor, BH1750_HALF_LX);
		
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

void BH1750_ReadOnceHighResolutionData(BH1750 *sensor, BH1750_MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		BH1750_ReadOnceHighResolutionData_Mode1(sensor); //Высокая точность, 1 люкс
	}else{
		BH1750_ReadOnceHighResolutionData_Mode2(sensor); //Высокая точность, 0.5 люкс
	}
}

BH1750_STATUS BH1750_ReadOnceLowResolutionData(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_ONCE_LOW_RES_MODE;
	
	//Отправляем команду
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, sensor->buff, 1, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	//Ждём конца измерений, время вычисления определим динамически
	HAL_Delay(0.45 * sensor->sensitivity);
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive(sensor->port, sensor->address, sensor->buff, 2, HAL_MAX_DELAY);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	//Записываем результат в структуру в сыром виде. Требует превращения в люксы
	BH1750_GetLux(sensor, BH1750_ONE_LX);
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS BH1750_SensorPowerSwitch(BH1750 *sensor, uint8_t mode){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_ResetSensor(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_SetSensorSensitivity(BH1750 *sensor, uint8_t sensitivity){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Подготовка сообщения в буфере
	sensor->buff[0] = 0x40 | ((sensitivity >> 5) & 0x07);
	sensor->buff[1] = 0x60 | (sensitivity & 0x1F);
	
	//Отправка команды для старших битов MTreg (формат: 01000_MT[7,6,5])
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, &sensor->buff[0], 1, HAL_MAX_DELAY);
	if(*sensor->ret != HAL_OK) {
			sensor->lastError = BH1750_TX_ERROR;
			return BH1750_TX_ERROR;
	}

	//Отправка команды для младших битов MTreg (формат: 011_MT[4,3,2,1,0])
	*sensor->ret = HAL_I2C_Master_Transmit(sensor->port, sensor->address, &sensor->buff[1], 1, HAL_MAX_DELAY);
	if(*sensor->ret != HAL_OK) {
			sensor->lastError = BH1750_TX_ERROR;
			return BH1750_TX_ERROR;
	}
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->sensitivity = sensitivity;
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

void BH1750_Deleter(BH1750 *sensor){
	free(sensor->buff);
	free(sensor->ret);
	free(sensor);
}
/////////////////////////////NON-BLOCKING MODE WITH INTERRUPTIONS/////////////////////////////
BH1750_STATUS BH1750_TransmitHighResolutionData_Mode1_IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_TransmitHighResolutionData_Mode2_IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

void BH1750_TransmitHighResolutionData_IT(BH1750 *sensor, BH1750_MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		BH1750_TransmitHighResolutionData_Mode1_IT(sensor); //Высокая точность, 1 люкс
	}else{
		BH1750_TransmitHighResolutionData_Mode2_IT(sensor); //Высокая точность, 0.5 люкс
	}
}


BH1750_STATUS BH1750_TransmitLowResolutionData_IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_TransmitOnceHighResolutionData_Mode1_IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_TransmitOnceHighResolutionData_Mode2_IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_ONCE_HIGH_RES_MODE2;

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

void BH1750_TransmitOnceHighResolutionData_IT(BH1750 *sensor, BH1750_MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		BH1750_TransmitOnceHighResolutionData_Mode1_IT(sensor); //Высокая точность, 1 люкс
	}else{
		BH1750_TransmitOnceHighResolutionData_Mode2_IT(sensor); //Высокая точность, 0.5 люкс
	}
}

BH1750_STATUS BH1750_TransmitOnceLowResolutionData_IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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


BH1750_STATUS BH1750_SensorPowerSwitch_IT(BH1750 *sensor, uint8_t mode){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_ResetSensor_IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_Recieve_IT(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive_IT(sensor->port, sensor->address, sensor->buff, 2);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS BH1750_SetSensorSensitivity_IT(BH1750 *sensor, uint8_t sensitivity){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Подготовка сообщения в буфере
	sensor->buff[0] = 0x40 | ((sensitivity >> 5) & 0x07);
	sensor->buff[1] = 0x60 | (sensitivity & 0x1F);
	
	//Отправка команды для старших битов MTreg (формат: 01000_MT[7,6,5])
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, &sensor->buff[0], 1);

	//Отправка команды для младших битов MTreg (формат: 011_MT[4,3,2,1,0])
	*sensor->ret = HAL_I2C_Master_Transmit_IT(sensor->port, sensor->address, &sensor->buff[1], 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->sensitivity = sensitivity;
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}


/////////////////////////////NON-BLOCKING MODE WITH DMA/////////////////////////////
BH1750_STATUS BH1750_TransmitHighResolutionData_Mode1_DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_TransmitHighResolutionData_Mode2_DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

void BH1750_TransmitHighResolutionData_DMA(BH1750 *sensor, BH1750_MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		BH1750_TransmitHighResolutionData_Mode1_DMA(sensor); //Высокая точность, 1 люкс
	}else{
		BH1750_TransmitHighResolutionData_Mode2_DMA(sensor); //Высокая точность, 0.5 люкс
	}
}


BH1750_STATUS BH1750_TransmitLowResolutionData_DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_TransmitOnceHighResolutionData_Mode1_DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_TransmitOnceHighResolutionData_Mode2_DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Указываем команду
	sensor->buff[0] = BH1750_ONCE_HIGH_RES_MODE2;

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

void BH1750_TransmitOnceHighResolutionData_DMA(BH1750 *sensor, BH1750_MEASUREMENT_RESOLUTION_MODE mode){
	//Вызов функции измерения в зависимости от режима
	if(mode == BH1750_ONE_LX){
		BH1750_TransmitOnceHighResolutionData_Mode1_DMA(sensor); //Высокая точность, 1 люкс
	}else{
		BH1750_TransmitOnceHighResolutionData_Mode2_DMA(sensor); //Высокая точность, 0.5 люкс
	}
}

BH1750_STATUS BH1750_TransmitOnceLowResolutionData_DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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


BH1750_STATUS BH1750_SensorPowerSwitch_DMA(BH1750 *sensor, uint8_t mode){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_ResetSensor_DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
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

BH1750_STATUS BH1750_Recieve_DMA(BH1750 *sensor){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Запрашиваем результат
	*sensor->ret = HAL_I2C_Master_Receive_DMA(sensor->port, sensor->address, sensor->buff, 2);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_RX_ERROR;
		return BH1750_RX_ERROR;
	}
	
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}

BH1750_STATUS BH1750_SetSensorSensitivity_DMA(BH1750 *sensor, uint8_t sensitivity){
	//Проверка заполнения указателей структуры
	if(BH1750_IsNull(sensor)){
		return BH1750_NOT_INITIALIZED;
	}
	
	//Подготовка сообщения в буфере
	sensor->buff[0] = 0x40 | ((sensitivity >> 5) & 0x07);
	sensor->buff[1] = 0x60 | (sensitivity & 0x1F);
	
	//Отправка команды для старших битов MTreg (формат: 01000_MT[7,6,5])
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, &sensor->buff[0], 1);

	//Отправка команды для младших битов MTreg (формат: 011_MT[4,3,2,1,0])
	*sensor->ret = HAL_I2C_Master_Transmit_DMA(sensor->port, sensor->address, &sensor->buff[1], 1);
	
	//Если получили ошибку - выход из функции
	if(*sensor->ret != HAL_OK){
		sensor->lastError = BH1750_TX_ERROR;
		return BH1750_TX_ERROR;
	}	
	
	sensor->sensitivity = sensitivity;
	sensor->lastError = BH1750_OK;
	return BH1750_OK;
}