# @Author: Enrique Heredia Aguado <enheragu>
# @Date:   12-Sep-2017
# @Project: RHA
# @Last modified by:   enheragu
# @Last modified time: 12-Sep-2017

# TORQUE_OFFSET = 70

n_data = 150
speed_target = 80
regulatorTest = [['16.66'],['40','110','1353']\
,['37','113','1471']\
,['34','116','1589']\
,['37','113','1708']\
,['42','108','1830']\
,['50','100','1948']\
,['46','104','2066']\
,['43','107','2185']\
,['41','109','2308']\
,['41','109','2430']\
,['42','108','2548']\
,['43','107','2667']\
,['44','106','2790']\
,['45','105','2913']\
,['44','106','3036']\
,['43','107','3153']\
,['44','106','3272']\
,['46','104','3390']\
,['45','105','3513']\
,['47','103','3631']\
,['45','105','3749']\
,['45','105','3867']\
,['40','110','3985']\
,['43','107','4104']\
,['43','107','4227']\
,['45','105','4349']\
,['49','101','4467']\
,['42','108','4586']\
,['42','108','4709']\
,['43','107','4832']\
,['45','105','4955']\
,['45','105','5078']\
,['46','104','5196']\
,['46','104','5314']\
,['45','105','5437']\
,['50','100','5560']\
,['47','103','5678']\
,['44','106','5796']\
,['44','106','5914']\
,['40','110','6037']\
,['49','101','6156']\
,['47','103','6279']\
,['46','104','6402']\
,['47','103','6524']\
,['44','106','6642']\
,['44','106','6761']\
,['41','109','6884']\
,['43','107','7007']\
,['43','107','7130']\
,['42','108','7247']\
,['43','107','7366']\
,['45','105','7484']\
,['44','106','7607']\
,['45','105','7725']\
,['44','106','7843']\
,['43','107','7961']\
,['42','108','8084']\
,['40','110','8202']\
,['40','110','8321']\
,['43','107','8443']\
,['43','107','8561']\
,['43','107','8679']\
,['42','108','8798']\
,['44','106','8921']\
,['42','108','9043']\
,['41','109','9161']\
,['44','106','9280']\
,['42','108','9403']\
,['44','106','9526']\
,['47','103','9649']\
,['49','101','9766']\
,['48','102','9885']\
,['48','102','10003']\
,['47','103','10126']\
,['44','106','10245']\
,['47','103','10368']\
,['49','101','10490']\
,['46','104','10608']\
,['46','104','10727']\
,['54','96','10850']\
,['49','101','10973']\
,['53','97','11090']\
,['44','106','11209']\
,['37','113','11327']\
,['43','107','11445']\
,['45','105','11564']\
,['46','104','11686']\
,['46','104','11804']\
,['46','104','11923']\
,['43','107','12046']\
,['45','105','12169']\
,['45','105','12292']\
,['44','106','12410']\
,['43','107','12528']\
,['42','108','12651']\
,['44','106','12770']\
,['40','110','12893']\
,['43','107','13016']\
,['41','109','13138']\
,['40','110','13256']\
,['46','104','13375']\
,['41','109','13498']\
,['43','107','13616']\
,['45','105','13734']\
,['44','106','13852']\
,['41','109','13970']\
,['41','109','14089']\
,['43','107','14212']\
,['42','108','14334']\
,['43','107','14453']\
,['44','106','14571']\
,['43','107','14694']\
,['47','103','14812']\
,['46','104','14930']\
,['43','107','15048']\
,['44','106','15166']\
,['44','106','15285']\
,['42','108','15408']\
,['45','105','15531']\
,['47','103','15649']\
,['46','104','15767']\
,['47','103','15890']\
,['44','106','16008']\
,['44','106','16126']\
,['44','106','16244']\
,['41','109','16362']\
,['44','106','16481']\
,['45','105','16604']\
,['49','101','16727']\
,['45','105','16845']\
,['56','94','16963']\
,['42','108','17086']\
,['44','106','17204']\
,['46','104','17323']\
,['40','110','17445']\
,['46','104','17563']\
,['45','105','17682']\
,['43','107','17805']\
,['47','103','17928']\
,['43','107','18045']\
,['42','108','18164']\
,['41','109','18282']\
,['46','104','18400']\
,['44','106','18519']\
,['45','105','18641']\
,['45','105','18759']\
,['49','101','18878']\
,['41','109','19001']\
,['44','106','19124']\
,['43','107','19241']\
,['37','113','19360']\
]

n_data2 = 150
speed_target2 = 80
regulatorTest2 = [['16.66'],['41','109','20851']\
,['34','116','20969']\
,['34','116','21088']\
,['34','116','21211']\
,['40','110','21334']\
,['48','102','21456']\
,['46','104','21575']\
,['44','106','21693']\
,['47','103','21816']\
,['45','105','21934']\
,['45','105','22052']\
,['48','102','22170']\
,['48','102','22288']\
,['46','104','22407']\
,['46','104','22530']\
,['46','104','22652']\
,['39','111','22771']\
,['42','108','22889']\
,['43','107','23012']\
,['53','97','23130']\
,['51','99','23248']\
,['49','101','23366']\
,['46','104','23484']\
,['47','103','23603']\
,['45','105','23726']\
,['43','107','23848']\
,['50','100','23966']\
,['45','105','24085']\
,['45','105','24208']\
,['44','106','24331']\
,['48','102','24450']\
,['47','103','24572']\
,['45','105','24690']\
,['45','105','24808']\
,['42','108','24927']\
,['43','107','25050']\
,['41','109','25172']\
,['44','106','25291']\
,['43','107','25409']\
,['42','108','25532']\
,['44','106','25650']\
,['41','109','25768']\
,['41','109','25886']\
,['43','107','26004']\
,['43','107','26123']\
,['44','106','26246']\
,['44','106','26369']\
,['42','108','26487']\
,['42','108','26605']\
,['43','107','26728']\
,['43','107','26846']\
,['41','109','26964']\
,['42','108','27082']\
,['43','107','27200']\
,['39','111','27319']\
,['42','108','27442']\
,['45','105','27565']\
,['43','107','27683']\
,['46','104','27801']\
,['44','106','27924']\
,['46','104','28042']\
,['47','103','28161']\
,['45','105','28283']\
,['47','103','28401']\
,['45','105','28520']\
,['44','106','28643']\
,['46','104','28766']\
,['46','104','28883']\
,['47','103','29002']\
,['45','105','29120']\
,['47','103','29238']\
,['45','105','29357']\
,['47','103','29479']\
,['47','103','29597']\
,['44','106','29716']\
,['46','104','29839']\
,['44','106','29962']\
,['48','102','30080']\
,['45','105','30198']\
,['39','111','30316']\
,['41','109','30434']\
,['41','109','30553']\
,['46','104','30675']\
,['47','103','30793']\
,['46','104','30912']\
,['46','104','31035']\
,['48','102','31158']\
,['40','110','31276']\
,['41','109','31394']\
,['43','107','31512']\
,['39','111','31630']\
,['46','104','31749']\
,['43','107','31872']\
,['43','107','31989']\
,['44','106','32108']\
,['44','106','32231']\
,['43','107','32354']\
,['42','108','32473']\
,['43','107','32595']\
,['44','106','32713']\
,['45','105','32831']\
,['46','104','32950']\
,['46','104','33073']\
,['44','106','33196']\
,['43','107','33314']\
,['44','106','33432']\
,['44','106','33555']\
,['44','106','33673']\
,['43','107','33792']\
,['43','107','33914']\
,['44','106','34032']\
,['46','104','34151']\
,['49','101','34274']\
,['45','105','34397']\
,['48','102','34514']\
,['45','105','34633']\
,['42','108','34751']\
,['43','107','34869']\
,['42','108','34988']\
,['40','110','35110']\
,['56','94','35228']\
,['46','104','35347']\
,['53','97','35470']\
,['49','101','35593']\
,['37','113','35710']\
,['48','102','35829']\
,['38','112','35947']\
,['42','108','36070']\
,['46','104','36189']\
,['46','104','36312']\
,['50','100','36434']\
,['46','104','36552']\
,['45','105','36671']\
,['44','106','36794']\
,['46','104','36917']\
,['45','105','37036']\
,['41','109','37158']\
,['39','111','37276']\
,['42','108','37394']\
,['45','105','37513']\
,['46','104','37636']\
,['47','103','37758']\
,['43','107','37876']\
,['41','109','37995']\
,['41','109','38118']\
,['43','107','38241']\
,['43','107','38360']\
,['44','106','38482']\
,['45','105','38600']\
,['44','106','38718']\
,['39','111','38837']\
]
