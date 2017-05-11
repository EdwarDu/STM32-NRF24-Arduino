uint8_t nRF_data[8] = {00, 00, 00, 00, 00};
	uint8_t nRF_dlast = '0';
	float dist_in_mm;

	
	//nRF24_GotoRXMode();
	//nRF24_SendData(nRF_data, 8);


/* HC_SR04 Test Code Snippet
		dist_in_mm = hc_sr04_Get_Dist(1);
		if (dist_in_mm > 0) {
			printf("Dist is %.2f mm \r\n", dist_in_mm);
			delay_ms(60);
		} else {
			printf("Invalid Measurement\r\n");
		}*/
		
		/* NRF24 Receive Test Code Snippet
		if (nRF24_state.rx_dsize != 0){
			printf("Got %d :", nRF24_state.rx_dsize);
			for (i = 0; i< nRF24_state.rx_dsize; i ++){
				putchar(nRF24_state.rx_data[i]);
			}
			
			printf("\r\n");
			nRF24_GotoRXMode();
		} else {
			putchar('.');
			delay_msus(100, 0);
		}*/
		
		/* NRF24 Send Test Code Snippet
		if (nRF24_state.tx_data == NULL){
			printf("C");
			delay_ms(1000);
			for (i = 0; i < 8; i ++){
				nRF_data[i] ++;
			}
			nRF24_SendData(nRF_data, 8);
		} else if (nRF24_state.tx_data == (uint8_t *) -1){
			delay_ms(1000);
			for (i = 0; i < 8; i ++){
				nRF_data[i] ++;
			}
			nRF24_SendData( nRF_data, 8);
			if (15 == nRF24_GetPLossCnt()){
				nRF24_SetRFCh(nRF24_state.ch);
				printf("X");
			} else {
				printf("L");
			}
		} else {
			// putchar('.');
			delay_msus(10,0);
		}*/

		/*
		printf("PLOS: %d\tARC: %d\r\n", nRF24_GetPLossCnt(), nRF24_GetARCCnt());
		if (nRF24_GetPLossCnt() == 15){
			ch = nRF24_GetRFCh();
			ch = (ch+1) % 0x7F;
			nRF24_SetRFCh(ch);
			printf("Changing CH to %d\r\n", ch);
		}*/

		float vol = adc1_value * 5 * 3.3f / 4096;
		printf("%d %.4f\r\n", adc1_value, vol);
		getchar();