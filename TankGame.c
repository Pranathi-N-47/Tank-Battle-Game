#include "stm32f4xx.h"
#include <string.h> // For memset

// --- Configuration ---
#define SLAVE_ADDRESS_LCD       0x4E    // 8-bit I2C address for the LCD backpack
#define I2C_TIMEOUT             10000   // Timeout loops for I2C communication
#define ADC_TIMEOUT             10000   // Timeout loops for ADC conversion
#define UART_TIMEOUT            10000   // Timeout loops for UART transmission
#define DEBOUNCE_MS             50      // Debounce period for button presses

// PCF8574 Pin definitions (for the I2C backpack)
#define RS                      (1 << 0) // Register Select pin
#define RW                      (1 << 1) // Read/Write pin (not used, tied to GND)
#define EN                      (1 << 2) // Enable pin
#define BL                      (1 << 3) // Backlight pin

// Game Configuration
#define GAME_SPEED_MS           150     // Main game loop update interval
#define BULLET_SPEED            2       // How many columns a bullet moves per update
#define WINS_REQUIRED           3       // Number of hits needed to win a round
#define FREEZE_DURATION_MS      2000    // Duration a tank is frozen after being hit
#define BLINK_COUNT             4       // Number of times a tank blinks when frozen
#define HIT_INDICATOR_MS        1000    // How long the "HIT!" message stays on screen
#define COLLISION_MSG_MS        2000    // How long "Bullet Collision!" stays on screen

// Custom Character addresses in LCD CGRAM
#define CHAR_TANK_LEFT          0
#define CHAR_TANK_RIGHT         1
#define CHAR_BULLET             2

// --- Game State Structures ---
// Holds all data for a single tank
typedef struct {
    uint8_t row;            // Current row (0-3) on the LCD
    uint8_t col;            // Current column (0-19) on the LCD
    uint8_t health;         // Remaining health
    uint8_t frozen;         // 1 if frozen (invincible), 0 otherwise
    uint32_t freeze_start;  // Timestamp when the freeze began
    uint8_t visible;        // 1 if visible, 0 if invisible (for blinking)
    uint8_t hit_indicator_active;   // Flag to show "HIT!" message
    uint32_t hit_indicator_start; // Timestamp for "HIT!" message
} Tank;

// Holds all data for a single bullet
typedef struct {
    uint8_t active;         // 1 if the bullet is currently on screen
    uint8_t row;            // Current row of the bullet
    int8_t col;             // Current column of the bullet
    int8_t direction;       // Movement direction: 1 for right, -1 for left
} Bullet;

// A single structure to hold the entire state of the game
typedef struct {
    Tank tank1;
    Tank tank2;
    Bullet bullet1;
    Bullet bullet2;
    uint8_t game_over;      // Flag indicating the game has ended
    uint8_t winner;         // Stores the winner (1 or 2)
    uint8_t collision_indicator_active; // Flag for the "Bullet Collision!" message
    uint32_t collision_indicator_start; // Timestamp for the collision message
} GameState;

// --- Global Variables ---
volatile uint32_t systick_ticks = 0;        // Incremented every ms by the SysTick timer
volatile uint8_t button_pressed_flag = 0;   // Flag set by button interrupts (1 for P1, 2 for P2)
volatile uint32_t last_button_press_time = 0; // Timestamp for button debouncing
GameState game;                             // The global instance of the game state

// Screen buffers for robust, flicker-free drawing
char screen_buffer[4][21];      // The "next frame" to be drawn
char last_screen_buffer[4][21]; // A copy of what's currently on the LCD

// Default player names
char player1_name[16] = "Player 1";
char player2_name[16] = "Player 2";

// --- Helper Functions ---
/**
  * @brief  A simple blocking delay based on the SysTick timer.
  * @param  ms: The number of milliseconds to wait.
  * @retval None
  */
void delay_ms(uint32_t ms) {
    uint32_t start_tick = systick_ticks;
    while ((systick_ticks - start_tick) < ms) {
        // Busy wait
    }
}

/**
  * @brief  A lightweight integer-to-string conversion function.
  * @param  value: The integer to convert.
  * @param  str: A pointer to the character buffer to store the result.
  * @retval None
  */
void itoa_custom(int value, char *str) {
    char temp[6];
    int i = 0;
    if (value == 0) {
        str[0] = '0';
        str[1] = '\0';
        return;
    }
    int n = value;
    while (n > 0) {
        temp[i++] = (n % 10) + '0';
        n /= 10;
    }
    int j = 0;
    for (int k = i - 1; k >= 0; k--) {
        str[j++] = temp[k];
    }
    str[j] = '\0';
}

// -------------------- I2C1 Initialization and Driver --------------------
/**
  * @brief  Configures I2C1 peripheral and GPIO pins (PB6, PB7).
  * @retval None
  */
void I2C1_Init(void) {
    // Enable clocks for GPIOB and I2C1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;

    // Configure PB6 (SCL) and PB7 (SDA) for I2C1
    GPIOB->MODER &= ~((3 << 12) | (3 << 14));
    GPIOB->MODER |= ((2 << 12) | (2 << 14));    // Alternate function mode
    GPIOB->OTYPER |= (1 << 6) | (1 << 7);       // Open-drain output
    GPIOB->OSPEEDR |= (3 << 12) | (3 << 14);    // High speed
    GPIOB->PUPDR |= (1 << 12) | (1 << 14);      // Pull-up resistors
    GPIOB->AFR[0] |= (4 << 24) | (4 << 28);     // Set AF4 for I2C1

    // Reset and configure I2C1 for 100kHz standard mode
    I2C1->CR1 |= I2C_CR1_SWRST;
    I2C1->CR1 &= ~I2C_CR1_SWRST;

    I2C1->CR2 = 16;     // PCLK1 frequency (16MHz)
    I2C1->CCR = 80;     // Clock control register for 100kHz
    I2C1->TRISE = 17;   // Rise time register
    I2C1->CR1 |= I2C_CR1_PE; // Enable I2C1
}

/**
  * @brief  Writes multiple bytes to an I2C slave device with a timeout.
  * @param  addr: The 8-bit slave address.
  * @param  data: Pointer to the data buffer.
  * @param  len: Number of bytes to write.
  * @retval 0 on success, 1 on timeout/failure.
  */
uint8_t I2C1_WriteMulti(uint8_t addr, uint8_t *data, uint8_t len) {
    volatile uint32_t timeout;

    // Wait until the bus is not busy
    timeout = I2C_TIMEOUT;
    while (I2C1->SR2 & I2C_SR2_BUSY) {
        if ((timeout--) == 0) {
            return 1;
        }
    }

    // Generate START condition
    I2C1->CR1 |= I2C_CR1_START;
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_SB)) {
        if ((timeout--) == 0) {
            return 1;
        }
    }

    // Send slave address
    I2C1->DR = addr;
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_ADDR)) {
        if ((timeout--) == 0) {
            return 1;
        }
    }
    (void)I2C1->SR2; // Clear ADDR flag

    // Send data bytes
    for (uint8_t i = 0; i < len; i++) {
        timeout = I2C_TIMEOUT;
        while (!(I2C1->SR1 & I2C_SR1_TXE)) {
            if ((timeout--) == 0) {
                return 1;
            }
        }
        I2C1->DR = data[i];
    }

    // Wait for transfer to complete
    timeout = I2C_TIMEOUT;
    while (!(I2C1->SR1 & I2C_SR1_BTF)) {
        if ((timeout--) == 0) {
            return 1;
        }
    }

    // Generate STOP condition
    I2C1->CR1 |= I2C_CR1_STOP;
    return 0; // Success
}

// -------------------- LCD Driver Functions --------------------
/**
  * @brief  Writes a 4-bit nibble to the LCD (used for initialization).
  * @retval 0 on success, 1 on failure.
  */
uint8_t LCD_Write_Nibble(uint8_t nibble) {
    uint8_t buffer[2];
    buffer[0] = nibble | BL | EN; // Pulse Enable HIGH
    buffer[1] = nibble | BL;      // Pulse Enable LOW
    return I2C1_WriteMulti(SLAVE_ADDRESS_LCD, buffer, 2);
}

/**
  * @brief  Sends a full byte (command or data) to the LCD in 4-bit mode.
  * @retval 0 on success, 1 on failure.
  */
uint8_t LCD_Send(uint8_t data, uint8_t flags) {
    uint8_t high_nibble = data & 0xF0;
    uint8_t low_nibble = (data << 4) & 0xF0;
    uint8_t buffer[4];

    // Send high nibble
    buffer[0] = high_nibble | flags | BL | EN;
    buffer[1] = high_nibble | flags | BL;
    // Send low nibble
    buffer[2] = low_nibble | flags | BL | EN;
    buffer[3] = low_nibble | flags | BL;

    return I2C1_WriteMulti(SLAVE_ADDRESS_LCD, buffer, 4);
}

/**
  * @brief  Sends a command to the LCD.
  * @retval 0 on success, 1 on failure.
  */
uint8_t LCD_SendCmd(uint8_t cmd) {
    return LCD_Send(cmd, 0); // RS=0 for command
}

/**
  * @brief  Sends a data character to the LCD.
  * @retval 0 on success, 1 on failure.
  */
uint8_t LCD_SendData(uint8_t data) {
    return LCD_Send(data, RS); // RS=1 for data
}

/**
  * @brief  Initializes the LCD controller in 4-bit mode.
  * @retval 0 on success, 1 on failure.
  */
uint8_t LCD_Init(void) {
    uint8_t status = 0;
    delay_ms(50); // Wait for LCD power up

    // --- 4-bit Initialization Sequence ---
    status |= LCD_Write_Nibble(0x30);
    delay_ms(5);
    status |= LCD_Write_Nibble(0x30);
    delay_ms(1);
    status |= LCD_Write_Nibble(0x30);
    delay_ms(1);
    status |= LCD_Write_Nibble(0x20); // Set 4-bit mode
    delay_ms(1);

    if (status) {
        return 1;
    }

    // --- Standard Configuration ---
    status |= LCD_SendCmd(0x28); // Function Set: 4-bit, 2-line, 5x8 font
    status |= LCD_SendCmd(0x0C); // Display Control: Display ON, Cursor OFF
    status |= LCD_SendCmd(0x06); // Entry Mode Set: Increment cursor
    status |= LCD_SendCmd(0x01); // Clear display
    delay_ms(5); // Clear command takes longer

    return status;
}

/**
  * @brief  Creates a custom character in the LCD's CGRAM.
  * @param  loc: CGRAM location (0-7).
  * @param  charmap: Pointer to an 8-byte array defining the character.
  */
void LCD_CreateCustomChar(uint8_t loc, uint8_t charmap[8]) {
    loc &= 0x7; // Ensure location is within 0-7
    LCD_SendCmd(0x40 | (loc << 3)); // Set CGRAM address
    for (int i = 0; i < 8; i++) {
        LCD_SendData(charmap[i]);
    }
}

/**
  * @brief  Moves the LCD cursor to a specified row and column.
  */
void LCD_SetCursor(uint8_t row, uint8_t col) {
    uint8_t address;
    switch (row) {
        case 0: {
            address = 0x80 + col;
            break;
        }
        case 1: {
            address = 0xC0 + col;
            break;
        }
        case 2: {
            address = 0x94 + col;
            break;
        }
        case 3: {
            address = 0xD4 + col;
            break;
        }
        default: {
            address = 0x80 + col;
        }
    }
    LCD_SendCmd(address);
}

/**
  * @brief  Prints a string to the LCD at the current cursor position.
  */
void LCD_Print(char *str) {
    while (*str) {
        LCD_SendData(*str++);
    }
}

/**
  * @brief  Clears the entire LCD display.
  */
void LCD_Clear(void) {
    LCD_SendCmd(0x01);
    delay_ms(5);
}

// -------------------- ADC Driver --------------------
/**
  * @brief  Configures ADC1 and GPIO pins for joystick inputs.
  */
void ADC_Init(void) {
    // Enable clocks for GPIOA and ADC1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Configure PA0, PA1, PA2, PA3 as analog inputs
    GPIOA->MODER |= (3 << (0 * 2)) | (3 << (1 * 2)) | (3 << (2 * 2)) | (3 << (3 * 2));

    // Configure ADC1
    ADC->CCR = 0;
    ADC1->CR1 = 0;
    ADC1->CR2 = ADC_CR2_ADON; // Enable ADC
    delay_ms(2); // Wait for ADC to stabilize
}

/**
  * @brief  Reads a single value from a specified ADC channel with a timeout and reset recovery.
  * @param  channel: The ADC channel to read (0-15).
  * @retval 12-bit ADC value, or center value (2048) on timeout.
  */
uint16_t ADC_Read(uint8_t channel) {
    volatile uint32_t timeout = ADC_TIMEOUT;

    // It's good practice to clear status flags before starting a conversion
    ADC1->SR = 0;

    ADC1->SQR3 = channel; // Set channel for conversion
    ADC1->CR2 |= ADC_CR2_SWSTART; // Start conversion

    // Wait for End of Conversion flag
    while (!(ADC1->SR & ADC_SR_EOC)) {
        if ((timeout--) == 0) {
            // --- Robust Error Recovery ---
            // If the ADC hangs (times out), a simple return isn't enough. The
            // peripheral might be in an error state. We need to reset it to
            // ensure the next conversion can succeed.
            ADC1->CR2 &= ~ADC_CR2_ADON; // Disable ADC
            delay_ms(1);                // Small delay for power down
            ADC1->CR2 |= ADC_CR2_ADON;  // Re-enable ADC
            delay_ms(2);                // Wait for stabilization
            return 2048;                // Return a neutral value for this failed attempt
        }
    }
    // Reading the DR register automatically clears the EOC flag.
    return ADC1->DR; // Return converted value
}

// -------------------- UART Driver (Transmit Only) --------------------
/**
  * @brief  Configures USART1 for transmitting debug/score messages.
  */
void UART_Init(void) {
    // Enable clocks for GPIOA and USART1
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    // Configure PA9 (TX) and PA10 (RX) for USART1
    GPIOA->MODER &= ~((3 << (9 * 2)) | (3 << (10 * 2)));
    GPIOA->MODER |= (2 << (9 * 2)) | (2 << (10 * 2)); // Alternate function
    GPIOA->AFR[1] &= ~((0xF << (1 * 4)) | (0xF << (2 * 4)));
    GPIOA->AFR[1] |= (7 << (1 * 4)) | (7 << (2 * 4)); // AF7 for USART1

    // Configure USART1 for 9600 baud
    USART1->BRR = 0x683;
    // Enable UART and Transmitter (Receiver is disabled)
    USART1->CR1 = USART_CR1_UE | USART_CR1_TE;
}

/**
  * @brief  Sends a single character via UART with a timeout.
  */
void UART_SendChar(char c) {
    volatile uint32_t timeout = UART_TIMEOUT;
    while (!(USART1->SR & USART_SR_TXE)) {
        if ((timeout--) == 0) {
            return;
        }
    }
    USART1->DR = c;
}

/**
  * @brief  Sends a null-terminated string via UART.
  */
void UART_SendString(char *str) {
    while (*str) {
        UART_SendChar(*str++);
    }
}

/**
  * @brief  Converts a number to a string and sends it via UART.
  */
void UART_SendNumber(uint32_t num) {
    char buf[12];
    itoa_custom(num, buf);
    UART_SendString(buf);
}

// -------------------- Button and Interrupt Initialization --------------------
/**
  * @brief  Configures GPIO pins and EXTI lines for joystick buttons.
  */
void Peripherals_Init(void) {
    // Enable clocks for GPIOB and SYSCFG
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;

    // Configure PB0 and PB1 as inputs with pull-ups
    GPIOB->MODER &= ~((3 << 0) | (3 << 2));
    GPIOB->PUPDR |= (1 << 0) | (1 << 2);

    // Map EXTI lines to GPIOB
    SYSCFG->EXTICR[0] &= ~(SYSCFG_EXTICR1_EXTI0 | SYSCFG_EXTICR1_EXTI1);
    SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PB | SYSCFG_EXTICR1_EXTI1_PB;

    // Configure EXTI lines for interrupt on falling edge
    EXTI->IMR |= (1 << 0) | (1 << 1);
    EXTI->FTSR |= (1 << 0) | (1 << 1);

    // Enable interrupts in the NVIC
    NVIC_EnableIRQ(EXTI0_IRQn);
    NVIC_EnableIRQ(EXTI1_IRQn);
}

/**
  * @brief  Interrupt handler for EXTI line 0 (Player 1 button).
  */
void EXTI0_IRQHandler(void) {
    if (EXTI->PR & (1 << 0)) {
        // Debounce and set flag
        if (systick_ticks - last_button_press_time > DEBOUNCE_MS) {
            button_pressed_flag = 1;
            last_button_press_time = systick_ticks;
        }
        EXTI->PR |= (1 << 0); // Clear pending bit
    }
}

/**
  * @brief  Interrupt handler for EXTI line 1 (Player 2 button).
  */
void EXTI1_IRQHandler(void) {
    if (EXTI->PR & (1 << 1)) {
        // Debounce and set flag
        if (systick_ticks - last_button_press_time > DEBOUNCE_MS) {
            button_pressed_flag = 2;
            last_button_press_time = systick_ticks;
        }
        EXTI->PR |= (1 << 1); // Clear pending bit
    }
}

// -------------------- System Timer (SysTick) --------------------
/**
  * @brief  Configures the SysTick timer for a 1ms interrupt.
  */
void SysTick_Init(void) {
    SysTick->LOAD = 16000 - 1; // 1ms tick @ 16MHz
    SysTick->VAL = 0;
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_TICKINT_Msk | SysTick_CTRL_ENABLE_Msk;
}

/**
  * @brief  SysTick interrupt handler, increments the global tick counter.
  */
void SysTick_Handler(void) {
    systick_ticks++;
}

// -------------------- Custom Characters Graphics --------------------
/**
  * @brief  Defines and loads the custom character graphics into the LCD.
  */
void LCD_InitCustomChars(void) {
    // Tank Left (facing right) - Custom Char 0
    uint8_t tank_left[8] = {
        0b00111,
        0b01111,
        0b11100,
        0b11111,
        0b11111,
        0b11100,
        0b01111,
        0b00111
    };

    // Tank Right (facing left) - Custom Char 1
    uint8_t tank_right[8] = {
        0b11100,
        0b11110,
        0b00111,
        0b11111,
        0b11111,
        0b00111,
        0b11110,
        0b11100
    };

    // Bullet - Custom Char 2
    uint8_t bullet[8] = {
        0b00000,
        0b00100,
        0b01110,
        0b01110,
        0b01110,
        0b01110,
        0b00100,
        0b00000
    };

    LCD_CreateCustomChar(CHAR_TANK_LEFT, tank_left);
    LCD_CreateCustomChar(CHAR_TANK_RIGHT, tank_right);
    LCD_CreateCustomChar(CHAR_BULLET, bullet);
}

// -------------------- GAME LOGIC FUNCTIONS --------------------
/**
  * @brief  Resets the game state to its initial values for a new round.
  */
void Game_Init(void) {
    // Reset Player 1
    game.tank1.row = 1;
    game.tank1.col = 0;
    game.tank1.health = WINS_REQUIRED;
    game.tank1.frozen = 0;
    game.tank1.visible = 1;
    game.tank1.hit_indicator_active = 0;

    // Reset Player 2
    game.tank2.row = 1;
    game.tank2.col = 19;
    game.tank2.health = WINS_REQUIRED;
    game.tank2.frozen = 0;
    game.tank2.visible = 1;
    game.tank2.hit_indicator_active = 0;

    // Reset bullets
    game.bullet1.active = 0;
    game.bullet2.active = 0;

    // Reset game state
    game.game_over = 0;
    game.winner = 0;
    game.collision_indicator_active = 0;

    // Clear screen buffers
    memset(screen_buffer, ' ', sizeof(screen_buffer));
    // FIX: Initialize last_screen_buffer with a value that won't be on screen (like 0xFF)
    // This forces the renderer to draw the initial state of the game correctly.
    memset(last_screen_buffer, 0xFF, sizeof(last_screen_buffer));
}

/**
  * @brief  Handles the freeze and blinking logic for tanks that have been hit.
  */
void Game_UpdateFreezeStates(void) {
    uint32_t elapsed;
    uint32_t blink_interval = FREEZE_DURATION_MS / (BLINK_COUNT * 2);

    // Handle Tank 1
    if (game.tank1.frozen) {
        elapsed = systick_ticks - game.tank1.freeze_start;
        if (elapsed >= FREEZE_DURATION_MS) {
            game.tank1.frozen = 0;
            game.tank1.visible = 1;
        } else {
            // Toggle visibility based on time to create a blink effect
            game.tank1.visible = ((elapsed / blink_interval) % 2 == 0);
        }
    }

    // Handle Tank 2
    if (game.tank2.frozen) {
        elapsed = systick_ticks - game.tank2.freeze_start;
        if (elapsed >= FREEZE_DURATION_MS) {
            game.tank2.frozen = 0;
            game.tank2.visible = 1;
        } else {
            game.tank2.visible = ((elapsed / blink_interval) % 2 == 0);
        }
    }
}

/**
  * @brief  Checks and deactivates the temporary "HIT!" and "Collision" indicators.
  */
void Game_UpdateIndicators(void) {
    // Check Tank 1's hit indicator
    if (game.tank1.hit_indicator_active) {
        if (systick_ticks - game.tank1.hit_indicator_start >= HIT_INDICATOR_MS) {
            game.tank1.hit_indicator_active = 0;
        }
    }

    // Check Tank 2's hit indicator
    if (game.tank2.hit_indicator_active) {
        if (systick_ticks - game.tank2.hit_indicator_start >= HIT_INDICATOR_MS) {
            game.tank2.hit_indicator_active = 0;
        }
    }
    
    // Check the bullet collision indicator
    if (game.collision_indicator_active) {
        if (systick_ticks - game.collision_indicator_start >= COLLISION_MSG_MS) {
            game.collision_indicator_active = 0;
        }
    }
}

/**
  * @brief  Reads joystick inputs and button presses to update tank positions and fire bullets.
  */
void Game_ProcessInput(void) {
    // Add a small delay between ADC reads to improve stability, especially
    // during fast joystick movements which can introduce noise.
    uint16_t joy1y = ADC_Read(1);
    delay_ms(1);
    uint16_t joy2y = ADC_Read(3);

    // Player 1 Movement (Up/Down)
    if (!game.tank1.frozen) {
        if (joy1y < 1200 && game.tank1.row > 0) {
            game.tank1.row--;
        } else if (joy1y > 3000 && game.tank1.row < 3) {
            game.tank1.row++;
        }
    }

    // Player 2 Movement (Up/Down)
    if (!game.tank2.frozen) {
        if (joy2y < 1200 && game.tank2.row > 0) {
            game.tank2.row--;
        } else if (joy2y > 3000 && game.tank2.row < 3) {
            game.tank2.row++;
        }
    }

    // Player 1 Fire
    if (button_pressed_flag == 1 && !game.bullet1.active && !game.tank1.frozen) {
        game.bullet1.active = 1;
        game.bullet1.row = game.tank1.row;
        game.bullet1.col = 1;
        game.bullet1.direction = 1;
    }

    // Player 2 Fire
    if (button_pressed_flag == 2 && !game.bullet2.active && !game.tank2.frozen) {
        game.bullet2.active = 1;
        game.bullet2.row = game.tank2.row;
        game.bullet2.col = 18;
        game.bullet2.direction = -1;
    }

    button_pressed_flag = 0; // Clear the flag after processing
}

/**
  * @brief  Updates bullet positions and checks for collisions.
  */
void Game_UpdateBullets(void) {
    // Update Player 1's bullet
    if (game.bullet1.active) {
        game.bullet1.col += game.bullet1.direction * BULLET_SPEED;

        // Check for hit on Player 2
        if (game.bullet1.col >= 19 && game.bullet1.row == game.tank2.row && !game.tank2.frozen) {
            game.tank2.health--;
            game.bullet1.active = 0;
            game.tank2.frozen = 1;
            game.tank2.freeze_start = systick_ticks;
            game.tank2.hit_indicator_active = 1;
            game.tank2.hit_indicator_start = systick_ticks;

            // Send updated score via UART
            UART_SendString("\r\nPlayer 1 hits Player 2 | Lives: ");
            UART_SendNumber(game.tank1.health);
            UART_SendString(" - ");
            UART_SendNumber(game.tank2.health);

            // Check for game over
            if (game.tank2.health == 0) {
                game.game_over = 1;
                game.winner = 1;
            }
        } else if (game.bullet1.col >= 20) {
            // Bullet went off-screen
            game.bullet1.active = 0;
        }
    }

    // Update Player 2's bullet
    if (game.bullet2.active) {
        game.bullet2.col += game.bullet2.direction * BULLET_SPEED;

        // Check for hit on Player 1
        if (game.bullet2.col <= 0 && game.bullet2.row == game.tank1.row && !game.tank1.frozen) {
            game.tank1.health--;
            game.bullet2.active = 0;
            game.tank1.frozen = 1;
            game.tank1.freeze_start = systick_ticks;
            game.tank1.hit_indicator_active = 1;
            game.tank1.hit_indicator_start = systick_ticks;


            // Send updated score via UART
            UART_SendString("\r\nPlayer 2 hits Player 1 | Lives: ");
            UART_SendNumber(game.tank1.health);
            UART_SendString(" - ");
            UART_SendNumber(game.tank2.health);

            // Check for game over
            if (game.tank1.health == 0) {
                game.game_over = 1;
                game.winner = 2;
            }
        } else if (game.bullet2.col < 0) {
            // Bullet went off-screen
            game.bullet2.active = 0;
        }
    }

    // Check for bullet-on-bullet collision
    if (game.bullet1.active && game.bullet2.active && game.bullet1.row == game.bullet2.row && game.bullet1.col >= game.bullet2.col) {
        game.bullet1.active = 0;
        game.bullet2.active = 0;
        
        // Activate collision indicator and message
        game.collision_indicator_active = 1;
        game.collision_indicator_start = systick_ticks;
        
        // Freeze both tanks
        game.tank1.frozen = 1;
        game.tank1.freeze_start = systick_ticks;
        game.tank2.frozen = 1;
        game.tank2.freeze_start = systick_ticks;
        
        UART_SendString("\r\nBullets Collided!");
    }
}

/**
  * @brief  Draws the current game state into the screen buffer array.
  */
void Game_UpdateScreenBuffer(void) {
    // Clear buffer with spaces
    memset(screen_buffer, ' ', sizeof(screen_buffer));

    // Draw tanks if they are visible
    if (game.tank1.visible) {
        screen_buffer[game.tank1.row][game.tank1.col] = CHAR_TANK_LEFT;
    }
    if (game.tank2.visible) {
        screen_buffer[game.tank2.row][game.tank2.col] = CHAR_TANK_RIGHT;
    }

    // Draw active bullets
    if (game.bullet1.active && game.bullet1.col < 20) {
        screen_buffer[game.bullet1.row][game.bullet1.col] = CHAR_BULLET;
    }
    if (game.bullet2.active && game.bullet2.col >= 0) {
        screen_buffer[game.bullet2.row][game.bullet2.col] = CHAR_BULLET;
    }

    // Draw "HIT!" indicators
    if (game.tank1.hit_indicator_active) {
        strncpy(&screen_buffer[game.tank1.row][game.tank1.col + 2], "HIT!", 4);
    }
    if (game.tank2.hit_indicator_active) {
        strncpy(&screen_buffer[game.tank2.row][game.tank2.col - 5], "HIT!", 4);
    }
    
    // Draw "Bullet Collision!" indicator
    if (game.collision_indicator_active) {
        strncpy(&screen_buffer[1][7], "Bullet", 6);
        strncpy(&screen_buffer[2][5], "Collision!", 10);
    }
}

/**
  * @brief  Renders the screen buffer to the physical LCD.
  * It only writes characters that have changed since the last frame.
  */
void Game_RenderScreen(void) {
    for (int r = 0; r < 4; r++) {
        for (int c = 0; c < 20; c++) {
            // If the character in the new buffer is different from the old one
            if (screen_buffer[r][c] != last_screen_buffer[r][c]) {
                // Move cursor and write the new character
                LCD_SetCursor(r, c);
                LCD_SendData(screen_buffer[r][c]);
            }
        }
    }
    // Copy the new buffer to the old buffer for the next frame's comparison
    memcpy(last_screen_buffer, screen_buffer, sizeof(screen_buffer));
}

/**
  * @brief  Displays the "Game Over" screen and the final thank you message.
  */
void Game_ShowGameOver(void) {
    // Clear the screen for the game over message
    LCD_Clear();

    // Display "GAME OVER!" and the winner on the LCD
    LCD_SetCursor(0, 5);
    LCD_Print("GAME OVER!");
    LCD_SetCursor(2, 2);
    LCD_Print((game.winner == 1 ? player1_name : player2_name));
    LCD_Print(" WINS!");

    // Send the final result via UART
    UART_SendString("\r\n\r\nGAME OVER! Winner: ");
    UART_SendString((game.winner == 1 ? player1_name : player2_name));
    UART_SendString("\r\n");

    // Wait for 3 seconds to show the winner
    delay_ms(3000);

    // Clear the screen for the final message
    LCD_Clear();

    // Set cursor to the requested position (row 1, column 1)
    LCD_SetCursor(1, 1);

    // Display the "Thanks for playing!" message
    LCD_Print("Thanks for playing!");

    // Halt the program in an infinite loop
    while(1) {
        // Game has ended. Do nothing.
    }
}

// -------------------- MAIN PROGRAM --------------------
int main(void) {
    uint32_t last_update = 0;

    // Initialize all hardware peripherals
    SysTick_Init();
    I2C1_Init();
    ADC_Init();
    Peripherals_Init();
    UART_Init();

    // Initialize LCD and check for errors
    if (LCD_Init() != 0) {
        // If LCD communication fails, halt indefinitely.
        while (1);
    }
    LCD_InitCustomChars();

    // Show startup sequence
    LCD_Clear();
    LCD_SetCursor(1, 5);
    LCD_Print("GET READY!");
    UART_SendString("\r\n\r\n--- TANK BATTLE ---\r\n");
    UART_SendString("Game Starting...\r\n");
    delay_ms(2000);
    LCD_Clear();

    // Initialize game state for the first round
    Game_Init();

    // --- Main Game Loop ---
    while (1) {
        // Lock game updates to a consistent speed
        if (systick_ticks - last_update >= GAME_SPEED_MS) {
            last_update = systick_ticks;

            if (!game.game_over) {
                // --- Gameplay Phase ---
                Game_UpdateFreezeStates();  // Handle invincibility blinking
                Game_UpdateIndicators();    // Handle "HIT!" and "Collision" message timers
                Game_ProcessInput();        // Read joysticks and buttons
                Game_UpdateBullets();       // Move bullets and check collisions
                Game_UpdateScreenBuffer();  // Draw new frame to buffer
                Game_RenderScreen();        // Render buffer to the physical LCD
            } else {
                // --- Game Over Phase ---
                Game_ShowGameOver();        // Display winner and end the game
            }
        }
    }
}

