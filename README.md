# ORB-Slam2_FeatureExtract
		1、为了使接下来计算PnP时能够更加准确，需要保持的原则就是尽量使得特征点能够均匀分布在整个图像中，
		所以需要将图像分成更小的部分分别提取特征点，并用某种机制来分配。在本项目中，作者提供了两种分配方案
		分别是八叉树的方法和传统的方法。考虑到在室内场景中并不需要提供太多的特征点，而在1000个特征点以下时
		，传统的方法表现要由于建造八叉树的方法。所以在实际应用中，都是采用传统的方法，而不去花费大量收时间
		用于建造八叉树。
    
		2、传统方法的第一步就是计算需要将图像分割成多少个元包（cell）,对于每个元包分别提取特征点。元包的计
		算方法为，根据需要提取的特征点数目，假设每个元包中需要提取5个特征点，以此来进行计算需要的cell数目。
   
		3、接着对上面计算好的元包分别进行特征点的提取。这里注意，由于FAST特征在计算角点时向内缩进了3个像素
		才开始计算，所以在使用FAST之前还需要对图像加一条宽度为3像素的边。然后就要用到之前初始化的两个阈值参
		数，首先使用阈值较大的参数作为FAST特征点检测的阈值，如果提取到的特征点数目足够多，那么直接计算下一个
		元包即可，否则就要使用较小的参数重新提取。在本项目中，特征点数目的阈值设定为3。
    
		4、然后就涉及到了特征点的数目分配问题。由于图像中不可避免的存在纹理丰富和纹理较浅的区域，在纹理较丰
		富的区域，角点的数目可能提取很多，而在纹理不丰富的区域，角点的数目可能很少。而在分配各区域选取的特征
		点数目时，就要考虑前面提到的极可能均匀的问题。所以采用的方法是循环将特征点数目不足的元包中的剩余数目
		分配到其他所有元包中，知道最后取得足够数量的特征点。当然，如果最初提取的特征点数目就不足预期，那么直
		接全部选取即可。所以这种方法并不能保证最终得到的特征点数目一定能达到1000。

		5、对于那些特征点数目特别多的元包，采用的是对各个角点的质量进行排序，选择最好的前n个特征点作为最终
		结果。
