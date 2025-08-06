# ms2-tb1-cakrai17-13624029

Berikut adalah node bridge publisher-subscriber untuk package magang_2025. Node ini bernama /node_zahran dimana node ini memiliki karakteristik sebagai berikut :
1. Melakukan subscribe kepada topik : autonomous_vel, joy_vel, dan keyboard_vel
2. Prioritas topic adalah sebagai berikut : keyboard_vel > joy_vel > autonomous_vel
3. Tidak menggunakan logika timer dalam membuat topik prioritas
4. User dapat menentukan topik mana yang akan disubscribe oleh node bridge (Special Feature)
5. Tanpa alasan yang jelas, data pasangan cmd_vel dan cmd_type pada node bridge memiliki sedikit perbedaan dengan data pasangan cmd_vel dan cmd_type pada node \movement_reader

# Penjelasan Karakteristik ke-4 :
User dapat memilih topik mana yang hendak diteruskan ke node /movement_reader. Pada file config.yaml, terdapat autonomous_status, joy_status, dan keyboard_status. Kontrol dengan status "True" menyatakan bahwa data twist dari kontrol tersebut dapat diteruskan ke node /movement_reader, sedangkan kontrol dengan status "False" tidak dapat meneruskan data twist ke node /movement_reader.

Pada settingan default, status dari ketiga kontrol (autonomous, joy, keyboard) adalah "True" sehingga data twist dari ketiga kontrol tersebut dapat diteruskan ke node /movement_reader. Berikut adalah test case untuk beberapa konfigurasi :
1. autonomous_status : True, joy_status : True, keyboard_status : False --> Output node /movement_reader hanya akan menampilkan log "AUTONOMOUS" atau "JOY"
2. autonomous_status : False, joy_status : True, keyboard_status : False --> Output node /movement_reader hanya akan menampilkan log "JOY" atau "UNKNOWN"

Jika user tidak mengaktifkan status autonomous (autonomous_status : False), maka ada kemungkinan output pada node /movement_reader akan memberikan log "UNKNOWN" karena tidak ada data twist yang terbaca. Hal ini terjadi karena topik joy_vel dan topik keyboard_vel tidak selalu memberikan data twist kepada node bridge
