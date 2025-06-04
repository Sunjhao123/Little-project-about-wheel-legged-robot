# gazebo_factory_simulation

### Dünyanın çalıştırılabilmesi için: 

-gazebo_factory_simulation dizinindeki "building_editor_models" ve "model_editor_models" klasörleri Gazebo Simülasyon ortamında 
kullanılacak objelerin model dosyalarını içermektedir. 
-Bu objelerin doğru şekilde yüklenebilmesi için Gazebo'nun başlatıldığı terminalde dizinleri tanıtılmalıdır(bashrc belgesine de yazılabilir). 

Aşağıdaki komut satırları ile objeler Gazebo'ya tanıtılır.

```
 $ source /usr/share/gazebo-8/setup.sh 
 $ export GAZEBO_MODEL_PATH=/home/{Your Computer Name}/{Your Folder Name}/model_editor_models:/home/{Your Computer Name}/{Your Folder Name}/building_editor_models
```

-"world" klasöründe Gazebo'da kullanılacak dünya belgeleri bulunmaktadır. 
-GAZEBO_MODEL_PATH doğru tanımlandığında sorunsuz açılacaklardır.
