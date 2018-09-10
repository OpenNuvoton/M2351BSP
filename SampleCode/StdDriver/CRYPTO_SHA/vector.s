;/*---------------------------------------------------------------------------------------------------------*/
;/*                                                                                                         */
;/* Copyright(c) 2010 Nuvoton Technology Corp. All rights reserved.                                         */
;/*                                                                                                         */
;/*---------------------------------------------------------------------------------------------------------*/


    AREA _image, DATA, READONLY

    EXPORT  g_u32VectorDataBase
    EXPORT  g_u32VectorDataLimit

    ALIGN   4
        
g_u32VectorDataBase
    INCBIN .\sha_test_vector
g_u32VectorDataLimit

    END