package frc.robot.tutorial.bank;

/**
 * An 'interface' describes what we expect to be
 * able to do with a class, i.e. how we would
 * 'interface' with it, which methods we want to be there.
 * 
 * For a bank account, we expect to be able to somehow
 * get the current balance,
 * make a deposit,
 * and withdraw money.
 */
public interface Account
{
    // A method for getting the current balance.
    // We would simply call this, without passing any argument in,
    // and we get the balance back as a 'double'
    
    /** @return Current account balance */
    public double getBalance();

    // A method for making a deposit.
    // We call it with a 'double' for the amount we want to deposit.
    // It doesn't return anything, 'void'.
    // If you want to know your balance after the deposit,
    // call getBalance()
    
    /** @param amount Money to put into the account */
    public void deposit(double amount);

    // The method to make a withdrawal takes an argument,
    // the amount you'd like to withdraw,
    // and returns a number to show what you actually got.
    // If your account doesn't have enough of a balance,
    // you might after all not get what you tried to withdraw.
    
    /** @param requested_amount Amount that I'd like to withdraw
     *  @return What I actually got
     */
    public double withdraw(double requested_amount);
}
