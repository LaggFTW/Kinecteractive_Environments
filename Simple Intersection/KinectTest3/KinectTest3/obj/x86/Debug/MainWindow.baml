   M S B A M L   `   `   `  ����^  ZMicrosoft.Samples.Kinect.WpfViewers, Version=0.0.0.0, Culture=neutral, PublicKeyToken=null�^clr-namespace:Microsoft.Samples.Kinect.WpfViewers;assembly=Microsoft.Samples.Kinect.WpfViewers#Microsoft.Samples.Kinect.WpfViewers  5   e    KinectTest3   KinectTest3.MainWindow   5   	   R NWindowsBase, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35W SPresentationCore, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35\ XPresentationFramework, Version=4.0.0.0, Culture=neutral, PublicKeyToken=31bf3856ad364e35D 9http://schemas.microsoft.com/winfx/2006/xaml/presentation    5   	   R NSystem.Xaml, Version=4.0.0.0, Culture=neutral, PublicKeyToken=b77a5c561934e089:x,http://schemas.microsoft.com/winfx/2006/xaml     5   	   mmy^clr-namespace:Microsoft.Samples.Kinect.WpfViewers;assembly=Microsoft.Samples.Kinect.WpfViewers     5   e   -   5          � Title$  
MainWindow��5   	   $	��600��6   $	��800��6)   .��5      �  � RowDefinitions 5   
   �� 5      $
��490* �6   �� 5      $
��111* �6   .+�5
   
   =   7Microsoft.Samples.Kinect.WpfViewers.KinectSensorChooser  -   5        $��kinectSensorChooser1��5
   R   $
��Left=�6!   $��1742,535,0,0q�6<   $	��Top=�6n   $	��328��6�   �� 5   
   -   5        $��image1��5   8   $	��480��6   $
��Left=�6   $
��Fill=�6F   $	��Top=�6U   $	��640��6m   $��1585,386,0,0q�6y   ;   5Microsoft.Samples.Kinect.WpfViewers.KinectColorViewer  5   
   -   5        $��kinectColorViewer1��5   O   $
��Left=�6   $��-338,59,0,0q�6:   $	��Top=�6i   $	��240��6�   $	��320��6�   5   /Microsoft.Samples.Kinect.WpfViewers.ImageViewer   Kinect 6�   ��  �� ElementName$ kinectSensorChooser1�� �� Path$ Kinect��� 5   
   -   5        $��textBox1��5   L   $��59��6   $
��Left=�6   $��	12,58,0,0q�69   $	��Top=�6\   $	��594��6t   $��1��6�   �� 5   
   -   5        $��textBox2��5   M   $��60��6   $
��Left=�6   $��
12,143,0,0q�69   $	��Top=�6]   $	��594��6u   $��1��6�   �� 5   
   -   5        $��image2��5   9   $
��1200��6   $
��Left=�6   $
��Fill=�6G   $	��Top=�6V   $
��3648��6n   $��0,0,0,0q�6{   $��2��6�   5      5      