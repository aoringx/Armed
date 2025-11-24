void xbee_set_high() {
    HAL_Delay(1000);
    char enter[] = "+++";
    HAL_UART_Transmit(&huart1, (uint8_t*)enter, 3, 100);
    HAL_Delay(1000);

    // D0 = 5 (digital output high)
    char cmd1[] = "ATD0 5\r";
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd1, strlen(cmd1), 100);

    // apply
    char cmd2[] = "ATAC\r";
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd2, strlen(cmd2), 100);

    // exit
    char cmd3[] = "ATCN\r";
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd3, strlen(cmd3), 100);
}

void xbee_set_low() {
    HAL_Delay(1000);
    char enter[] = "+++";
    HAL_UART_Transmit(&huart1, (uint8_t*)enter, 3, 100);
    HAL_Delay(1000);
    
    // D0 = 4 (digital output low)
    char cmd1[] = "ATD0 4\r";  // LOW
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd1, strlen(cmd1), 100);

    // apply
    char cmd2[] = "ATAC\r";
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd2, strlen(cmd2), 100);

    // exit
    char cmd3[] = "ATCN\r";
    HAL_UART_Transmit(&huart1, (uint8_t*)cmd3, strlen(cmd3), 100);
}