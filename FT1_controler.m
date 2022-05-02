function Am = FT1_controler(erro,r,L);


         %    {***********Região IC-1*****************}
     if ( erro>0  & r>0 ) 
         
         mi5 = (-r/L + 1)*(-erro/L + 1); mi6 = (-erro/L + 1)*(r/L); mi8 = (erro/L)*(-r/L + 1); mi9 = (r/L)*(erro/L); 
            
         Am = mi5*(1-exp(-mi5*4)) + mi6*(1-exp(-mi6*4)) + mi8*(exp(-mi8*4)) + mi9*(exp(-mi9*4));
         
     end;

%      {***********Região IC-2*****************}
      if ( erro<=0  & r<=0) 
         
         mi1 = (-r/L )*(-erro/L); mi2 = (-erro/L)*(r/L + 1); mi4 = (-r/L)*(erro/L + 1); mi5 = (r/L + 1)*(erro/L + 1); 
            
         Am = mi1*(exp(-mi1*4)) + mi2*(exp(-mi2*4)) + mi4*(1 - exp(-mi4*4)) + mi5*( 1 - exp(-mi5*4));  
      
      end;

 %     {***********Região IC-3*****************}
      if ( erro>0  & r<=0) 
      
           mi5 = (-r/L + 1)*(-erro/L + 1); mi4 = (-erro/L + 1)*(r/L); mi8 = (erro/L)*(-r/L + 1); mi7 = (-r/L)*(erro/L); 
            
           Am = mi5*(1-exp(-mi5*4))+mi4*(1-exp(-mi4*4))+mi8*(exp(-mi8*4))+mi7*(exp(-mi7*4));
      
      end;
      
 %      {***********Região IC-4*****************}
      if ( erro<=0  & r>0)
          
          mi5 = (-r/L + 1)*(erro/L + 1); mi6 = (erro/L + 1)*(r/L); mi2 = (-erro/L)*(-r/L + 1); mi3 = (-r/L)*(-erro/L); 
            
          Am = mi5*(1-exp(-mi5*4)) + mi6*(1-exp(-mi6*4)) + mi2*(exp(-mi2*4)) + mi3*(exp(-mi3*4));
      
      end