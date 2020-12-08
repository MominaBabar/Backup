print("Welcome to JAKE Bakers")
items=[]  #main list,all items dictionaries are appended in this
tru=[]
count=0
def ADD():
    count=len(items)
    print("ADD")
    item=input('press yes to continue and no to quit : ')
    if item== 'yes':
        print('Enter details for the items to be added')
        x=0
        name=str(input('enter name of new item : '))
        while(x==0):
            bool = name.isalpha()
            if bool:
                x+=1
            else:
                name=str(input('Wrong Entry! Enter Name of new Item Again : '))
        x=0
        flavour=str(input('enter flavour of new item : '))
        while(x==0):
            bool = flavour.isalpha()
            if bool:
                x+=1
            else:
                flavour=str(input('Wrong Entry! Enter Flavour of new Item Again : '))        
        x=0
        expiry=input('enter expiry date of new item : ')
        while(x==0):
            bool = expiry.isalpha()
            if not bool:
                x+=1
            else:
                expiry=str(input('Wrong Entry! Enter Expiry of new Item Again : '))        
        
        
        x=0
        price=str(input('enter price of new item : '))
        while(x==0):
            bool = price.isalpha()
            if bool:
                price=str(input('Wrong Entry! Enter Price of new Item Again '))
            else:
                price=int(price)
                if price>0 and price<=1000:
                    x+=1
                else:
                    price=str(input('Wrong Entry! Enter Price of new Item Again '))
        
    price=int(price)
    count+=1
    tru={"item no":count,"name":name,"price":price,"flavour":flavour,"expiry":expiry}
    items.append(tru)
    print(items)
    if item=='no':
        print('THANK YOU')
        
    
        
def EDIT():
    print("EDIT")
    n=input("press yes to edit an atttribute status in an item").upper()
    if n=="yes".upper():
        n2=input("Which Item To Edit? e.g item1").upper()
        if n2=="item1".upper():
            x=input("Enter Attribute").lower()
            if x=='name':
                value=input("Set new name value to:")
                items[0]['name']=value
                print(items)
                
            if x=='flavour':
                value=input("Set new flavour value to:")
                items[0]['flavour']=value
                print(items)
              
            if x=='price':
                value=input("Set new price value to:")
                items[0]['price']=value
                print(items)
                
            if x=='expiry':
                value=input("Set new expiry value to:")
                items[0]['expiry']=value
                print(items)
        if n2=="item2".upper():
            x=input("Enter Attribute").lower()
            if x=='name':
                value=input("Set new name value to:")
                items[1]['name']=value
                print(items)
                    
            if x=='flavour':
                value=input("Set new flavour value to:")
                items[1]['flavour']=value
                print(items)
                
            if x=='price':
                value=input("Set new price value to:")
                items[1]['price']=value
                print(items)
                
            if x=='expiry':
                value=input("Set new expiry value to:")
                items[1]['expiry']=value
                print(items)
        if n2=="item3".upper():
            x=input("Enter Attribute").lower()
            if x==name:
                value=input("Set new name value to:")
                items[2]['name']=value
                print(items)
                
            if x==flavour:
                value=input("Set new flavour value to:")
                items[2]['flavour']=value
                print(items)
                
            if x==price:
                value=input("Set new price value to:")
                items[2]['price']=value
                print(items)
               
            if x==expiry:
                value=input("Set new expiry value to:")
                items[2]['expiry']=value
                print(items)
        if n2=="item4".upper():
            x=input("Enter Attribute").lower()
            if x==name:
                value=input("Set new name value to:")
                items[3]['name']=value
                print(items)
            if x==flavour:
                value=input("Set new flavour value to:")
                items[3]['flavour']=value
                print(items)
            if x==price:
                value=input("Set new price value to:")
                items[3]['price']=value
                print(items)
               
            if x==expiry:
                value=input("Set new expiry value to:")
                items[3]['expiry']=value
                print(items)
def SEARCH(): #definition of search function
    print("SEARCH")
    n=int(input("enter the item no:"))
    for i in items:
        if i["item no"]==n:
            prop=input("Which property do you want to search?")
            if(prop=="name"):
                print(i["name"])
            elif(prop=="price"):
                print(i["price"])
            elif(prop=="flavour"):
                print(i["flavour"])
            elif(prop=="expiry"):
                print(i["expiry"])
        print('press y to search other property\npress n to quit')
            
        otherproperty=input('Do you want to search any other property?')
        
        if otherproperty=='y':
           while otherproperty=='y':
                prop=input("Which property do you want to search?")
                if(prop=="name"):
                    print(i["name"])
                elif(prop=="price"):
                    print(i["price"])
                elif(prop=="flavour"):
                    print(i["flavour"])
                elif(prop=="expiry"):
                    print(i["expiry"])
                otherproperty=input('Do you want to search any other property?')
        elif otherproperty=='n':
            print('thank you')
            
            
            
            return#the return will stop the function after we search the desire item or property of item,and the function will go back from where it is called
        
    print("This item does not exist")
def VIEW():
    print("VIEW")
    print(items)

count+=1
item1={   #dictionary of item1
    "item no":count,
    "name":"cake",
    "price":1000,
    "flavour":"straberry",
    "expiry":"10/13/20"
    }
items.append(item1)

count+=1   
item2={   #dictionary of item2
    "item no":count,
    "name":"pastery",
    "price":300,
    "flavour":"butterscotch",
    "expiry":"12/3/20"
    }
items.append(item2)

count+=1
item3={    #dictionary of item3
    "item no":count,
    "name":"donut",
    "price":60,
    "flavour":"chocholate",
    "expiry":"12,3,2020"
    }
items.append(item3)

count+=1    
item4={    #dictionary of item4
    "item no":count,
    "name":"cream roll",
    "price":100,
    "flavour":"vanilla",
    "expiry":"12/3/20"
    }
items.append(item4)
print(items)
while(True): #infinite loop
    
    choice=input("press a for ADD\npress ed for EDIT\npress s for SEARCH\npress v for VIEW\npress q for QUIT\nEnter choice : ")#enter choices records
    
    if (choice=="a"):#call to the ADD function
        ADD()
    elif(choice=="ed"):#call to the EDIT function
       EDIT()
    elif(choice=="s"):#call to the SEARCH function 
        SEARCH()
    elif(choice=="v"):#call to te VIEW function
        VIEW()
    elif(choice=="q"):#condition for QUIT,it will Quit the infinite loop after desired repitions
        break #break the loop after QUITING
