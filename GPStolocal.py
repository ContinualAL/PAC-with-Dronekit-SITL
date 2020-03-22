def GPStolocalr(phi1,long1,alt1,homelat,homelon,homealt):
                                                            import math
                                                           
                                                            #Ellipsodal model of Earth
                                                            ar=6378137  #meter (equatorial radius)
                                                            br= 6356752.3142   #meter (polar radius)

                                                            #Home point in lattitude longitude
                                                            phi0=homelat
                                                            long0=homelon           
                                                            alt0=homealt

                                                            

                                                            #Home point in ECEF (Earth frame)
                                                            P0 = [0,0,0]

                                                            N0=(ar*ar)/(math.sqrt(math.pow((ar*(math.cos(math.radians(phi0)))),2)+math.pow((br*(math.sin(math.radians(phi0)))),2)))

                                                            P0[0]=(N0+alt0)*(math.cos(math.radians(phi0)))*(math.cos(math.radians(long0)))
                                                            P0[1]=(N0+alt0)*(math.cos(math.radians(phi0)))*(math.sin(math.radians(long0)))
                                                            P0[2]=((math.pow((br/ar),2))*N0+alt0)*(math.sin(math.radians(phi0)))

                                                            #Point to be converted in lat-long (P1)
                                                            #phi1=38.897147
                                                            #long1=-77.043934
                                                            #alt1=940


                                                            #Point P1 converted to ECEF
                                                            Pe = [0,0,0]
                                                            N1=(ar*ar)/(math.sqrt(math.pow((ar*(math.cos(math.radians(phi1)))),2)+math.pow((br*(math.sin(math.radians(phi1)))),2)))

                                                            Pe[0]=(N1+alt1)*(math.cos(math.radians(phi1)))*(math.cos(math.radians(long1)))
                                                            Pe[1]=(N1+alt1)*(math.cos(math.radians(phi1)))*(math.sin(math.radians(long1)))
                                                            Pe[2]=((math.pow((br/ar),2))*N1+alt1)*(math.sin(math.radians(phi1)))

                                                            #Point P1 in Local frame (NED frame)
                                                            r11=-(math.sin(math.radians(phi1)))*(math.cos(math.radians(long1)))
                                                            r12=-(math.sin(math.radians(long1)))
                                                            r13=-(math.cos(math.radians(phi1)))*(math.cos(math.radians(long1)))

                                                            r21=-(math.sin(math.radians(phi1)))*(math.sin(math.radians(long1)))
                                                            r22=(math.cos(math.radians(long1)))
                                                            r23=-(math.cos(math.radians(phi1)))*(math.sin(math.radians(long1)))

                                                            r31=(math.cos(math.radians(phi1)))
                                                            r32=0
                                                            r33=-(math.sin(math.radians(phi1)))

                                                            #R = [ r11, r12, r13 ; r21, r22, r23 ; cosd(phi1) 0 -sind(phi1)]^T
                                                            Pa = [0,0,0]
                                                            Pa[0]=Pe[0]-P0[0]
                                                            Pa[1]=Pe[1]-P0[1]
                                                            Pa[2]=Pe[2]-P0[2]
                                                            #Pl = R*(Pe - P0);
                                                            Pl = [0,0,0]
                                                            Pl[0]=r11*Pa[0]+r21*Pa[1]+r31*Pa[2]
                                                            Pl[1]=r12*Pa[0]+r22*Pa[1]+r32*Pa[2]
                                                            Pl[2]=r13*Pa[0]+r23*Pa[1]+r33*Pa[2]

                                                            x = Pl[0]
                                                            y = Pl[1]
                                                            z = Pl[2]
                                                            #print x,y
                                                            

                                                            return x,y
