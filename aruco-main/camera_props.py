import numpy as np


camera_matrix = np.array([[625.55127378,   0.,         393.73177868],
                        [  0.,         626.57522214, 229.13859077],
                        [  0.,           0.,           1.        ]])

distortion_coefficient = np.array([[-0.04666253, -0.05100499, -0.00093192,  0.00162658,  0.16682155]])

rotation_vectors = np.array(([[-0.25938273],[ 0.0079747 ],[ 1.50074528]],
                             [[-0.72727678],[ 0.2096006 ],[ 0.30352687]],
                             [[-0.96242207],[ 0.11097239],[ 0.04514085]],
                             [[-0.23668805],[ 0.17981959],[ 1.56153276]],
                             [[-0.67169975],[-0.05660922],[-0.10349982]],
                             [[-0.8733009 ],[ 0.07861042],[ 0.02044558]],
                             [[ 0.30117026],[-0.33367812],[ 1.54648272]],
                             [[-0.69395573],[ 0.13225893],[ 0.29601962]],
                             [[ 0.07939623],[ 0.16005346],[-1.55818354]],
                             [[-0.73991628],[ 0.73005096],[ 1.4377085 ]],
                             [[-0.86395925],[ 0.28437042],[ 0.40012485]],
                             [[-0.21110544],[-0.01686135],[-1.50066617]]))

translation_vectors = np.array(([[ 1.51178084],[-3.07659341],[11.93076595]],
                                [[ 2.41766009],[-4.46883006],[14.92591666]],
                                [[-1.75345857],[-2.86384643],[13.11643668]],
                                [[ 4.56154747],[-2.52468439],[11.43856978]],
                                [[-1.64329018],[-3.68012426],[13.62092308]],
                                [[-2.21142044],[-3.49827784],[12.68581968]],
                                [[ 3.64273028],[-1.93173041],[ 9.13949828]],
                                [[ 0.33733772],[-4.43693973],[15.57876893]],
                                [[-3.34344083],[ 2.70423293],[11.72310043]],
                                [[ 3.6599063 ],[-1.20743223],[12.54156618]],
                                [[ 0.58656971],[-4.13942487],[14.63070116]],
                                [[-1.41341537],[ 1.74668208],[11.29403156]]))