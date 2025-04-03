Analizando el código, se están utilizando principalmente los puertos GPIOC y GPIOD. Aquí está el detalle:

**GPIOC (Entradas - Botones)**:
- PC0: Botón para incrementar
- PC1: Botón para decrementar
- PC2: Botón para cambiar de modo

**GPIOD (Salidas - LCD)**:
- PD0 a PD10: Configurados como salidas para la LCD
- PD8: Se usa como RS (Register Select)
- PD9: Se usa como E (Enable)

Operaciones específicas:
- Set (|=):
    ```c
    GPIOD->ODR |= (1 << 9);  // Set E (Enable)
    GPIOD->ODR |= (1 << 8);  // Set RS (Register Select)
    ```

- Reset (&= ~):
    ```c
    GPIOD->ODR &= ~(1 << 9); // Reset E (Enable)
    GPIOD->ODR &= ~(1 << 8); // Reset RS (Register Select)
    ```

Los puertos están configurados con:
- GPIOC: Configurado como entrada con pull-down
- GPIOD: Configurado como salida con alta velocidad y pull-down
