
# coding: utf-8

# # Parsimonious Autonomous Controller (PAC) simulation in Python code
# An evolving neuro-fuzzy controller with Hyerplane based membership function and has a significantly less network parameters
# 

# In[1]:


import math
import numpy as np


## Sigmoid Function code is self defined here:
def sigmf(X,A,C):
    return 1/(1 + math.exp(-A*(X-C)))

# Saturation limit function
def limiter(x,lim):
    if x >= lim:
        yout=lim
    elif x <= -1*lim:
        yout=-1*lim
    else:
        yout=x
    return yout

# ## PAC Function Class defined here:

# In[4]:


class structures:
    def __init__ (self):
        self.n = 5
        self.R = 1
        self.Rwin = 0
        self.itr = 1
        self.dt = 0.2
        self.varvar = 0
        self.varba = 0
        self.grow = 0
        self.prune = 0
        self.miuba = 0
        self.miubalast=0
        self.miuvar = 0
        self.miuvarlast=0
        self.miubamin = 0
        self.stdbamin = 0
        self.miuvarmin = 0
        self.stdvarmin = 0
        self.upalm = 0
        self.usrc = 0
        self.u = 0
        self.smcpar = [0.4,0.001,0.001] #tuned
        # self.smcpar = [0.2,0.0,0.0] 
        #self.smcpar = [1.55,0.0,0.0]
        self.Xe = np.asmatrix(np.ones((self.n,1)))
        self.w = 0.01*np.asmatrix(np.ones((self.R,self.n)))
        self.f = 0.01*np.asmatrix(np.eye(self.n*self.R,self.n))
        self.miuxe = np.asmatrix(np.zeros((self.n,1)))

    def process(self,err,derr,interr,y,yr):
        self.Xe=np.matrix('%s;%s;%s;%s;%s' %(1,err,derr,interr,y))
        # self.Xe=np.matrix('%s;%s;%s;%s;%s' %(1,err,derr,0.0,y))
        
        #distance (fuzzyfication) calc
        distden=np.zeros((self.R,1))
        dist=np.zeros((self.R,1))
        miub=np.zeros((self.R,1))
        for j in range(self.R):
            distden[j,0]=np.sum(np.multiply(self.w[j,:],self.w[j,:]))
            dist[j,0]=abs((yr-self.w[j,:]*self.Xe)/math.sqrt(1+distden[j,0]))
            miub[j,0]=math.exp(-0.5*(dist[j,0]/dist[j,0].max()))

        #inference rule calc
        lambdafuz=miub/np.sum(miub)
        psi=np.zeros((self.R,self.n))
        for j in range(self.R):
            psi[j,:]=lambdafuz[j,0]*self.Xe.transpose()

        #Upalm or defuzzyfication calc
        self.upalm=np.sum(np.multiply(psi,self.w))
        self.upalm=limiter(self.upalm,0.2)
        #Usrc calc
        sl=self.smcpar[0]*err+self.smcpar[1]*interr+self.smcpar[2]*derr
        #self.usrc=100*((2*sigmf(sl,4,0))-1)
        self.usrc=limiter(sl,0.8) #Linear saturation
        self.u=self.usrc-1.0*self.upalm

        #Rule Significance Calc
        self.miuxe+=(self.Xe-self.miuxe)/self.itr
        RS=abs(self.w*self.miuxe)
        self.Rwin=RS.argmax()
        
        #Winning Rule update mechanism
        psit=np.matrix(psi[self.Rwin,:])
        fwin=np.zeros((self.n,self.n))

        for i in range(self.n):
            for j in range(self.n):
                fwin[i,j]=self.f[(self.n*(self.Rwin-1)+i),j]
        
        fwin=np.matrix(fwin)
        wdot=-1*(fwin*psit.transpose())*sl
        fdot=-1*fwin*(psit.transpose()*psit)*fwin
        fwin=fwin+fdot*self.dt

        for i in range(self.n):
            for j in range(self.n):
                self.f[(self.n*(self.Rwin-1)+i),j]=fwin[i,j]
        #print(self.w[self.Rwin,:])
        #print(wdot.transpose()*self.dt)
        self.w[self.Rwin,:]=self.w[self.Rwin,:]+wdot.transpose()*self.dt

        # Bias Calc
        Ey=sum(self.w*self.miuxe)
        bias2=(yr-Ey)**2
        self.miuba+=(bias2-self.miuba)/self.itr
        self.varba+=(bias2-self.miubalast)*(bias2-self.miuba)
        stdba=math.sqrt(self.varba/self.itr)

        if self.itr==1 or self.grow==0:
            self.miubamin=self.miuba
            self.stdbamin=stdba

        # Rule Growing Mecha
        Xi=1*(1.5*math.exp(-1*(bias2))+0.5)
        RGleft=self.miuba+stdba
        RGright=self.miubamin+Xi*self.stdbamin
        if  RGleft >= RGright:
            self.R=self.R+1
            wnew=(self.miuxe/(abs(self.miuxe)).max()).transpose()*self.w[self.Rwin,:].mean()
            self.w=np.concatenate((self.w,wnew))
            self.f=np.concatenate((self.f,fwin))
            self.grow=1
            self.miubamin=self.miuba
            self.stdbamin=stdba
            #print('New rule has added')
        else:
            self.grow=0

        # Variance calc
        Ey2=sum(np.multiply(self.w,self.w)*(np.multiply(self.miuxe,self.miuxe)))
        variance=abs(Ey2-(Ey**2))
        self.miuvar+=(variance-self.miuvar)/self.itr
        self.varvar+=(variance-self.miuvarlast)*(variance-self.miuvar)
        stdvar=math.sqrt(self.varvar/self.itr)

        # Rule Pruning Mecha
        if self.itr>self.n+2 and self.R>=2:
            if self.prune==0:
                self.miuvarmin=self.miuvar
                self.stdvarmin=stdvar

            phii=1*(1.5*math.exp(-1*(variance))+0.5)
            # phii=1*(1.8*math.exp(-1*(variance))+0.2)
            # phii=1*(0.8*math.exp(-1*(variance))+1.2)
            left=self.miuvar+stdvar
            right=self.miuvarmin+phii*self.stdvarmin

            if left>=right:
                self.miuvarmin=self.miuvar
                self.stdvarmin=stdvar
                rulemin=np.argmin(RS)
                idx=0
                wupd=np.zeros((self.R-1,self.n))
                fupd=np.ones((self.n*(self.R-1),self.n))
                for j in range(self.R):
                    if j!=rulemin:
                        wupd[idx,:]=self.w[j,:]
                        for a in range(self.n):
                            for b in range(self.n):
                                fupd[self.n*(idx-1)+a,b]=fwin[a,b]
                        idx=idx+1
                self.w=wupd
                self.f=fupd
                self.R=self.R-1
                self.prune=1
                #print('Rule',rulemin,'has pruned')
        else:
            self.prune=0

        #save all latest data for next iteration 
        self.miubalast=self.miuba
        self.miuvarlast=self.miuvar

        #print('Iteration=',self.itr,', Number of Rules=',self.R,', Rule Winner=',self.Rwin,', Error=',err,', uself=',self.upalm,', usmc=',self.usrc)
        #print('---------------------------------------------------------------------------------------------------------------')
        self.itr=self.itr+1
