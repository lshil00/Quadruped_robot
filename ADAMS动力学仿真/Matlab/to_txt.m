function [] = to_txt(zqk_,zhk_,zqx_,zhx_,yqk_,yhk_,yqx_,yhx_)
fid=fopen('zqk.txt','w');
[b1 , ~]=size(zqk_);
for i=1:b1
   fprintf(fid,'%.2f %.2f \r\n',zqk_(i,1),zqk_(i,2));
end
fclose(fid);

fid=fopen('zhk.txt','w');
[b1, ~]=size(zhk_);
for i=1:b1
   fprintf(fid,'%.2f %.2f \r\n',zhk_(i,1),zhk_(i,2));
end
fclose(fid);

fid=fopen('zqx.txt','w');
[b1, ~]=size(zqx_);
for i=1:b1
   fprintf(fid,'%.2f %.2f \r\n',zqx_(i,1),zqx_(i,2));
end
fclose(fid);

fid=fopen('zhx.txt','w');
[b1, ~]=size(zhx_);
for i=1:b1
   fprintf(fid,'%.2f %.2f \r\n',zhx_(i,1),zhx_(i,2));
end
fclose(fid);

fid=fopen('yqk.txt','w');
[b1 , ~]=size(yqk_);
for i=1:b1
   fprintf(fid,'%.2f %.2f \r\n',yqk_(i,1),yqk_(i,2));
end
fclose(fid);

fid=fopen('yhk.txt','w');
[b1, ~]=size(yhk_);
for i=1:b1
   fprintf(fid,'%.2f %.2f \r\n',yhk_(i,1),yhk_(i,2));
end
fclose(fid);

fid=fopen('yqx.txt','w');
[b1, ~]=size(yqx_);
for i=1:b1
   fprintf(fid,'%.2f %.2f \r\n',yqx_(i,1),yqx_(i,2));
end
fclose(fid);

fid=fopen('yhx.txt','w');
[b1, ~]=size(yhx_);
for i=1:b1
   fprintf(fid,'%.2f %.2f \r\n',yhx_(i,1),yhx_(i,2));
end
fclose(fid);
end

