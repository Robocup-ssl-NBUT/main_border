# 					乐博车嵌入式烧录流程

## 一.需要的工具

​		1.stlinkv2，淘宝链接：[ST-LINK V2...-tmall.com天猫](https://detail.tmall.com/item.htm?id=721142227129&ali_refid=a3_430673_1006:1303560141:N:MXngtKJe48FnLBBMs3rZaA%3D%3D:a3c982211afb044e926e470a3f023497&ali_trackid=1_a3c982211afb044e926e470a3f023497&spm=a2e0b.20350158.31919782.6&bxsign=tanHISA8ogqubwspLzIcAK4y_8h3OPa93BHgkPXh5b5VLb9P8uT0E9Tup0jODBncSB3vo-YIon9UKVEA02CbfZbdJv3khD65_Cii0fHDJ8W1qk)

​		2.ph2.0转杜邦线 5pin双头排母一根：[PH2.0mm转杜邦线2.54mm双头连接线一拖2P3P4P5P6P转接测试端子线-淘宝网 (taobao.com)](https://item.taobao.com/item.htm?app=chrome&bxsign=scdj2FSl05uOotPH52vR4iFUCqba_9tb9eCuxQNGEX6Do8D_gAczq3RibGIjcIThb9T81WrpZeD6TJv-jjgfag0PHDgYg8kidlhlZnauGnaJaoyWGXPqpj7BzeJA9fqhExt&cpp=1&id=681787142664&price=2.4-7.2&shareUniqueId=26893633518&share_crt_v=1&shareurl=true&short_name=h.gWw14y0R10XzUwK&skuId=5056983487022&sourceType=item,item&sp_tk=VnpmVFd3NXFmTHE=&spm=a2159r.13376460.0.0&suid=ff3fcc17-0df3-40c4-9beb-7064260fa425&tbSocialPopKey=shareItem&tk=VzfTWw5qfLq&un=73098ffb6eb1e43c3431ceaac3991f67&un_site=0&ut_sk=1.YzEMXF0ACzIDAAJ7KpPkmW1r_21646297_1716893034437.Copy.1)

​		3.MDK-Keil5软件及stm32f4xx芯片库的安装：[Windows10下MDK5（Keil5）及stm32pack的安装教程_mdk 510-CSDN博客](https://blog.csdn.net/qq_45264808/article/details/108899662?ops_request_misc=%7B%22request%5Fid%22%3A%22171688395716777224422395%22%2C%22scm%22%3A%2220140713.130102334..%22%7D&request_id=171688395716777224422395&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~sobaiduend~default-1-108899662-null-null.142^v100^pc_search_result_base3&utm_term=windows下载keil5&spm=1018.2226.3001.4187)

		百度网盘：链接：https://pan.baidu.com/s/1YvAI46W11vh2upXxcAD_9g 提取码：bv3p

​		4.嵌入式源码：

​			github地址：https://github.com/Robocup-ssl-NBUT/main_board.git

​			gitee地址：https://gitee.com/fan-jiaming123/main_board.git

​		5.STM32 ST-LINK Utility软件----用于解除车上的烧写代码的保护机制

​	

